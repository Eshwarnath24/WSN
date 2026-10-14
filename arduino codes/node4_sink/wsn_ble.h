// ==================== WSN BLE LOCALIZATION LAYER ====================
// NimBLE-based BLE advertising, scanning, RSSI distance estimation,
// and localization report generation. Runs on ALL 4 nodes.
//
// Include after wsn_common.h and wsn_mesh.h.
// Requires NimBLE library (included in ESP32 Arduino Core 3.x).
// ====================================================================
#ifndef WSN_BLE_H
#define WSN_BLE_H

#include "wsn_common.h"
#include <NimBLEDevice.h>

// ==================== BLE CONFIGURATION ====================
// Manufacturer ID (arbitrary, used to identify our WSN packets)
#define WSN_BLE_MFG_ID  0xFFFF

// BLE advertising payload: [node_id (1 byte)] [tx_power_1m (1 byte as int8_t)]
// Total manufacturer data: 2 bytes company ID + 2 bytes payload = 4 bytes

// Pre-configured TRUE positions for each node (in meters, 2D)
// Adjust these to match your physical lab layout tape-measured positions.
const float NODE_TRUE_POS[NUM_NODES + 1][2] = {
  {0.0f, 0.0f},   // Index 0: unused
  {0.0f, 0.0f},   // N1: (0, 0) — ADJUST to actual
  {2.0f, 0.0f},   // N2: (2, 0) — ADJUST to actual
  {1.0f, 1.5f},   // N3: (1, 1.5) — ADJUST to actual
  {1.0f, -1.0f},  // N4: (1, -1.0) — non-collinear with N1 & N2
};

// ==================== BLE STATE ====================
static NimBLEAdvertising* pAdvertising = nullptr;
static NimBLEScan*         pScan = nullptr;
static bool ble_initialized = false;

// ==================== SCAN CALLBACK ====================
class WsnScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    // Check for our manufacturer data
    if (!device->haveManufacturerData()) return;

    // NimBLE returns manufacturer data as std::string: [companyID_lo, companyID_hi, data...]
    // We need at least 4 bytes: 2 company ID + 2 payload
    std::string raw = device->getManufacturerData();
    if (raw.length() < 4) return;

    uint16_t companyId = (uint8_t)raw[0] | ((uint8_t)raw[1] << 8);
    if (companyId != WSN_BLE_MFG_ID) return;

    uint8_t node_id = (uint8_t)raw[2];
    int8_t  tx_power = (int8_t)raw[3];

    if (node_id < 1 || node_id > NUM_NODES || node_id == MY_NODE_ID) return;

    int rssi = device->getRSSI();

    // Update BLE neighbor table
    ble_neighbor_t& bn = mesh.ble[node_id];
    bn.node_id = node_id;
    bn.rssi_raw = rssi;
    bn.last_seen_ms = millis();
    bn.discovered = true;

    // EWMA filter on RSSI
    if (bn.rssi_filtered == 0) {
      bn.rssi_filtered = (float)rssi;  // First sample
    } else {
      bn.rssi_filtered = BLE_EWMA_ALPHA * (float)rssi +
                         (1.0f - BLE_EWMA_ALPHA) * bn.rssi_filtered;
    }

    // Log-distance path loss model: d = 10^((A - RSSI) / (10 * n))
    float A = (float)tx_power;  // Calibrated RSSI at 1 m
    float exponent = (A - bn.rssi_filtered) / (10.0f * BLE_PATH_LOSS_N);
    bn.distance_m = powf(10.0f, exponent);

    Serial.printf("[BLE] Discovered N%d: RSSI=%d filtered=%.1f dist=%.2f m\n",
                  node_id, rssi, bn.rssi_filtered, bn.distance_m);
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    Serial.printf("[BLE] Scan complete: %d devices found\n", results.getCount());
  }
};

static WsnScanCallbacks scanCallbacks;

// ==================== BLE INITIALIZATION ====================
void ble_init() {
  char name[16];
  snprintf(name, sizeof(name), "WSN_N%d", MY_NODE_ID);

  NimBLEDevice::init(name);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // Max TX power for best range

  // --- Setup Advertising ---
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pAdvertising = NimBLEDevice::getAdvertising();

  // Build manufacturer data: 2 bytes company ID + [node_id, tx_power_1m]
  char mfgBytes[4];
  mfgBytes[0] = (char)(WSN_BLE_MFG_ID & 0xFF);
  mfgBytes[1] = (char)((WSN_BLE_MFG_ID >> 8) & 0xFF);
  mfgBytes[2] = (char)MY_NODE_ID;
  mfgBytes[3] = (char)(int8_t)BLE_TX_POWER_1M;
  std::string mfgData(mfgBytes, 4);

  NimBLEAdvertisementData advData;
  advData.setName(name);
  advData.setManufacturerData(mfgData);

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setMinInterval(BLE_ADV_INTERVAL_MS / 0.625f);
  pAdvertising->setMaxInterval((BLE_ADV_INTERVAL_MS * 2) / 0.625f);
  pAdvertising->start();

  // --- Setup Scanner ---
  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(80);

  ble_initialized = true;
  Serial.printf("[BLE] Initialized: %s (TX power at 1m = %d dBm, n = %.1f)\n",
                name, BLE_TX_POWER_1M, BLE_PATH_LOSS_N);
}

// ==================== BLE SCAN (Non-blocking burst) ====================
void ble_scan() {
  if (!ble_initialized || !pScan) return;

  // Non-blocking scan for BLE_SCAN_DURATION_MS (NimBLE 2.x takes milliseconds)
  if (!pScan->isScanning()) {
    Serial.println("[BLE] Starting scan burst...");
    pScan->start(BLE_SCAN_DURATION_MS, false);  // duration in ms, non-blocking
  }
}

// ==================== NEAREST NODE ====================
uint8_t ble_nearest_node() {
  float min_dist = 999.0f;
  uint8_t nearest = 0;
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    if (!mesh.ble[i].discovered) continue;
    if (mesh.ble[i].distance_m < min_dist) {
      min_dist = mesh.ble[i].distance_m;
      nearest = i;
    }
  }
  return nearest;
}

// ==================== BUILD BLE REPORT ====================
// Creates a BLE_REPORT payload with distances to all discovered neighbors.
// Localization (trilateration) is performed at the sink (N4).
ble_report_payload_t ble_build_report() {
  ble_report_payload_t rpt;
  memset(&rpt, 0, sizeof(rpt));
  rpt.reporter_id = MY_NODE_ID;
  rpt.nearest_node_id = ble_nearest_node();  // Fix: report local nearest node
  rpt.num_neighbors = 0;

  for (int i = 1; i <= NUM_NODES && rpt.num_neighbors < 3; i++) {
    if (i == MY_NODE_ID) continue;
    if (!mesh.ble[i].discovered) continue;
    uint8_t idx = rpt.num_neighbors;
    rpt.entries[idx].node_id = i;
    rpt.entries[idx].rssi = mesh.ble[i].rssi_raw;
    rpt.entries[idx].distance_m = mesh.ble[i].distance_m;
    rpt.num_neighbors++;
  }

  // Set true position (pre-configured)
  rpt.true_x = NODE_TRUE_POS[MY_NODE_ID][0];
  rpt.true_y = NODE_TRUE_POS[MY_NODE_ID][1];

  // Estimated position is computed at the sink; set to 0 here
  rpt.est_x = 0;
  rpt.est_y = 0;
  rpt.error_m = 0;

  return rpt;
}

// ==================== TRILATERATION (Run at Sink N4) ====================
// Given distances from a target node to 3 anchors, estimate 2D position.
// Uses linearized least-squares approach.
//
// anchor_pos[3][2]: x,y of each anchor
// distances[3]:     estimated distance to each anchor
// out_x, out_y:     computed position
// Returns true if solution found.
bool ble_trilaterate(const float anchor_pos[3][2], const float distances[3],
                     float& out_x, float& out_y) {
  // Linearize: subtract equation of anchor[0] from anchor[1] and anchor[2]
  // Ax = b where A is 2x2, x = [x, y], b = 2x1
  float x1 = anchor_pos[0][0], y1 = anchor_pos[0][1], d1 = distances[0];
  float x2 = anchor_pos[1][0], y2 = anchor_pos[1][1], d2 = distances[1];
  float x3 = anchor_pos[2][0], y3 = anchor_pos[2][1], d3 = distances[2];

  // A = 2 * [[x2-x1, y2-y1], [x3-x1, y3-y1]]
  float A00 = 2 * (x2 - x1), A01 = 2 * (y2 - y1);
  float A10 = 2 * (x3 - x1), A11 = 2 * (y3 - y1);

  // b = [d1^2 - d2^2 + x2^2 - x1^2 + y2^2 - y1^2,
  //      d1^2 - d3^2 + x3^2 - x1^2 + y3^2 - y1^2]
  float b0 = d1*d1 - d2*d2 + x2*x2 - x1*x1 + y2*y2 - y1*y1;
  float b1 = d1*d1 - d3*d3 + x3*x3 - x1*x1 + y3*y3 - y1*y1;

  float det = A00 * A11 - A01 * A10;
  if (fabsf(det) < 1e-6f) {
    Serial.println("[BLE] Trilateration: singular matrix (anchors collinear?)");
    return false;
  }

  out_x = (A11 * b0 - A01 * b1) / det;
  out_y = (A00 * b1 - A10 * b0) / det;
  return true;
}

// ==================== PRINT BLE STATUS ====================
void ble_print_status() {
  Serial.println("\n====== BLE STATUS ======");
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    ble_neighbor_t& bn = mesh.ble[i];
    if (bn.discovered) {
      Serial.printf("  N%d: RSSI=%d filtered=%.1f dist=%.2f m (last %lu ms ago)\n",
                    i, bn.rssi_raw, bn.rssi_filtered, bn.distance_m,
                    millis() - bn.last_seen_ms);
    } else {
      Serial.printf("  N%d: not discovered\n", i);
    }
  }
  uint8_t nearest = ble_nearest_node();
  if (nearest > 0) {
    Serial.printf("  Nearest node: N%d (%.2f m)\n", nearest, mesh.ble[nearest].distance_m);
  }
  Serial.println("========================\n");
}

#endif // WSN_BLE_H
