// ==================== NODE 2: FLAME SENSOR + RELAY ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Sensor: IR Optical Flame Detection (Analog AO + Digital DO)
// Additional role: RELAY — forwards packets from N1 or N3 when needed
// Primary route: N2 → N3 → N4    Alternate: N2 → N4 (direct)
// ======================================================================

#define MY_NODE_ID 2
#if __has_include("wsn_common.h")
#include "wsn_common.h"
#include "wsn_mesh.h"
#include "wsn_ble.h"
#else
#include "../common/wsn_common.h"
#include "../common/wsn_mesh.h"
#include "../common/wsn_ble.h"
#endif

// ==================== SENSOR CONFIG ====================
#define FLAME_AO_PIN 32    // Analog flame intensity (0–4095)
#define FLAME_DO_PIN 16    // Digital threshold (0 = fire, 1 = normal)
#define SENSOR_READ_INTERVAL_MS 250

// Latest sensor readings
float flameAnalog = 0;
float flameDigital = 1;  // 1 = normal (no fire)
bool hasNewSensorData = false;

// ==================== TIMERS ====================
uint32_t lastSensorRead = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;

// ==================== APP CALLBACKS ====================
// Called when a packet with final_dst=N2 arrives (rare — N2 is not the sink).
// Also called when N2 receives data it needs to forward (handled by mesh layer).
void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  Serial.printf("[APP] Data arrived for N2: type=%s origin=N%d seq=%d\n",
    msgTypeStr(pkt.msg_type), pkt.origin_id, pkt.seq_num);
}

// Called when data transits through us (intermediate hop). Return true to consume.
bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  return false;  // N2 is a relay — forward normally
}

// Called on CTRL commands
void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {
  // Collision test handled automatically by mesh_collision_test_send()
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 2 (Flame Sensor + Relay) BOOTING =====");

  // 1. Initialize sensor pins
  pinMode(FLAME_DO_PIN, INPUT);
  Serial.println("[SENSOR] Flame sensor pins configured (AO=GPIO32, DO=GPIO16)");

  // 2. Initialize BLE
  ble_init();

  // 3. Initialize mesh stack
  mesh_init();

  Serial.println("===== N2 SETUP COMPLETE =====\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t now = millis();

  // 1. Mesh periodic tasks (HELLO, timeouts, and RELAY slot forwarding)
  mesh_loop();

  // 2. Handle collision test trigger
  mesh_collision_test_send();

  // 3. BLE scan during sleep epoch
  tdma_epoch_t epoch = tdma_current_epoch();
  if (epoch == EPOCH_SLEEP && now - mesh.last_ble_scan_ms >= 3000) {
    ble_scan();
    mesh.last_ble_scan_ms = now;
  }

  // 4. Send BLE report periodically
  if (now - lastBleReport >= 10000) {
    lastBleReport = now;
    ble_report_payload_t rpt = ble_build_report();
    if (rpt.num_neighbors > 0) {
      mesh_send_ble_report(rpt);
    }
  }

  // 5. Read sensor (buffer latest sample)
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;

    flameAnalog = (float)analogRead(FLAME_AO_PIN);
    flameDigital = (float)digitalRead(FLAME_DO_PIN);
    hasNewSensorData = true;

    Serial.printf("[SENSOR] Flame: analog=%.0f digital=%.0f (%s)\n",
                  flameAnalog, flameDigital,
                  flameDigital == 0 ? "FIRE DETECTED!" : "Normal");
  }

  // 6. Transmit sensor data strictly during assigned TDMA TX slot (Defect 3 fix)
  static tdma_epoch_t last_tx_slot = EPOCH_SLEEP;
  static uint32_t last_tx_frame = 0xFFFFFFFF;
  uint32_t current_frame = mesh.tdma.frame_number;
  tdma_epoch_t curr_epoch = tdma_current_epoch();

  if (mesh.tdma.synced) {
    // Only transmit own sensor data during assigned TX slot (Slot 2)
    if (tdma_is_my_tx_slot()) {
      if (curr_epoch != last_tx_slot || current_frame != last_tx_frame) {
        if (hasNewSensorData && mesh.next_hop_to_sink != 0) {
          mesh_send_sensor_data(flameAnalog, flameDigital, 0, "flame");
          hasNewSensorData = false;
          last_tx_slot = curr_epoch;
          last_tx_frame = current_frame;
        }
      }
    } else {
      last_tx_slot = curr_epoch;
    }
  } else {
    // Free-running fallback when unsynced: transmit every 1s
    static uint32_t lastUnsyncedTx = 0;
    if (now - lastUnsyncedTx >= 1000) {
      lastUnsyncedTx = now;
      if (hasNewSensorData && mesh.next_hop_to_sink != 0) {
        mesh_send_sensor_data(flameAnalog, flameDigital, 0, "flame");
        hasNewSensorData = false;
      }
    }
  }

  // 7. Print status periodically
  if (now - lastStatusPrint >= 10000) {
    lastStatusPrint = now;
    mesh_print_status();
    ble_print_status();
  }
}
