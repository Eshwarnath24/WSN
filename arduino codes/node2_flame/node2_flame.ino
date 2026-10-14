// ==================== NODE 2: FLAME SENSOR + RELAY ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Sensor: IR Optical Flame Detection (Analog AO + Digital DO)
// Additional role: RELAY — forwards packets (e.g. N3's AGG when the N3–N4
// link fails) in its TDMA RELAY slot (S4) or in the CAP.
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
#define FLAME_AO_PIN 32    // Analog flame output (ADC1_CH4, 0–4095; usable while Wi-Fi is on)
#define FLAME_DO_PIN 16    // Digital comparator output (0 = fire, 1 = normal)
#define SENSOR_READ_INTERVAL_MS 250

// Latest sensor readings
float flameAnalog = 0;
float flameDigital = 1;  // 1 = normal (no fire)
bool hasNewSensorData = false;

// ==================== TIMERS ====================
uint32_t lastSensorRead = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;

// ==================== 5 s SUMMARY LINE ====================
void app_summary() {
  Serial.printf(" SENSOR: flame AO=%.0f DO=%.0f → %s | readings sent %u | relayed for others %lu\n",
                flameAnalog, flameDigital, flameDigital == 0 ? "FIRE DETECTED" : "normal",
                (unsigned)(mesh.data_seq_counter - 1), (unsigned long)mesh.pkts_forwarded);
}

// ==================== APP CALLBACKS ====================
void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  Serial.printf("[APP] Data arrived for N2: type=%s origin=N%d seq=%d\n",
    msgTypeStr(pkt.msg_type), pkt.origin_id, pkt.seq_num);
}

// Intermediate hop: N2 is a relay — forward normally (queued for RELAY slot / CAP)
bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  LOGV("[RELAY] Relaying %s from origin N%d seq=%d (prev hop N%d)\n",
                msgTypeStr(pkt.msg_type), pkt.origin_id, pkt.seq_num, pkt.src_id);
  return false;
}

void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {
  // Collision test is handled by mesh_collision_test_service()
}

// ==================== SETUP ====================
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 2 (Flame Sensor + Relay) BOOTING =====");

  // 1. Sensor pins
  pinMode(FLAME_DO_PIN, INPUT);
  Serial.println("[SENSOR] Flame sensor pins configured (AO=GPIO32, DO=GPIO16)");

  // 2. BLE + boot-time discovery
  ble_init();
  ble_boot_discovery();

  // 3. Mesh stack
  mesh_init();

  Serial.println("===== N2 SETUP COMPLETE =====\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t now = millis();

  // 1. Mesh periodic tasks (sync, HELLO, RELAY-slot forwarding)
  mesh_loop();

  // 2. Contention demo
  mesh_collision_test_service();

  // 3. BLE scan burst in SLEEP epoch
  ble_tick();
  ble_serial_poll();   // Serial Monitor: status | ble | cal <id> <m> | setn <n>

  // 4. BLE report every 10 s (CAP)
  if (now - lastBleReport >= BLE_REPORT_PERIOD_MS && !mesh_own_tx_busy(OWN_TX_BLE)) {
    lastBleReport = now;
    ble_report_payload_t rpt = ble_build_report();
    if (rpt.num_neighbors > 0) {
      mesh_own_tx_load(OWN_TX_BLE, mesh_build_ble_report_packet(rpt));
    }
  }
  if (mesh_own_tx_busy(OWN_TX_BLE) && mesh_ctrl_window_open()) {
    mesh_own_tx_service(OWN_TX_BLE);
  }

  // 5. Read sensor (buffer latest sample)
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;
    flameAnalog = (float)analogRead(FLAME_AO_PIN);
    flameDigital = (float)digitalRead(FLAME_DO_PIN);
    hasNewSensorData = true;
    static uint8_t printDiv = 0;
    if (++printDiv >= 4) {
      printDiv = 0;
      LOGV("[SENSOR] Flame: analog=%.0f digital=%.0f (%s)\n",
                    flameAnalog, flameDigital,
                    flameDigital == 0 ? "FIRE DETECTED!" : "Normal");
    }
  }

  // 6. TDMA: one new packet per assigned TX slot occurrence (S2)
  static uint32_t lastOpportunity = 0xFFFFFFFF;
  if (mesh_take_data_opportunity(lastOpportunity)) {
    {
      if (mesh_own_tx_busy(OWN_TX_DATA)) {
        // previous packet still pending → retransmitted below
      } else if (hasNewSensorData && mesh.next_hop_to_sink != 0) {
        mesh_own_tx_load(OWN_TX_DATA, mesh_build_sensor_packet(flameAnalog, flameDigital, 0, "flame"));
        hasNewSensorData = false;
        LOGV("[TDMA] %s: TX opportunity — sending flame reading seq=%d via N%d\n",
                      epochStr(tdma_current_epoch()), mesh.own_tx[OWN_TX_DATA].pkt.seq_num,
                      mesh.next_hop_to_sink);
      } else if (mesh.tdma.synced) {
        mesh_note_unused_slot(hasNewSensorData ? "no route to sink" : "no new sensor sample");
      }
    }
  }
  if (mesh_own_tx_busy(OWN_TX_DATA) && mesh_retry_window_open()) {
    mesh_own_tx_service(OWN_TX_DATA);
  }

  // 7. Compact 5 s status block (full detail: type 'status' or 'log 2')
  wsn_summary_tick();
}
