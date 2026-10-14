// ==================== NODE 3: CLUSTER HEAD / AGGREGATOR / IR SENSOR ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Sensor: IR Obstacle Sensor (Digital, GPIO 4)
// Additional roles:
//   - CLUSTER HEAD: computes, versions and broadcasts the TDMA schedule
//   - AGGREGATOR: collects N1 + N2 data, fuses into a single AGG packet
//   - ROUTER: forwards packets toward the sink (N4)
// Primary route: N3 → N4 (direct)    Alternate: N3 → N2 → N4 (or N3 → N1 → N4)
// ======================================================================================

#define MY_NODE_ID 3
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
// The guidelines require sensing on N1 and N2 only; N3's IR sensor is an
// optional extra. Set to 0 if no IR module is wired to GPIO4.
#ifndef N3_HAS_IR_SENSOR
#define N3_HAS_IR_SENSOR 1
#endif
#define IR_DO_PIN     4       // Digital obstacle sensor (active-low: LOW = obstacle)
#define DEBOUNCE_MS   50
#define SENSOR_READ_INTERVAL_MS 250

int lastRawReading = HIGH;
int stableState = HIGH;
uint32_t lastChangeTime = 0;
float irObstacle = 0;  // 1 = obstacle, 0 = clear

// ==================== AGGREGATION STATE ====================
// N1 sends in S1, N2 in S2; N3 sends one AGG in S3 every AGG_WINDOW_FRAMES
// superframes (earlier if the buffer is nearly full or N1 reports an anomaly).
portMUX_TYPE agg_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t lastN1AnomalyMs = 0;
volatile bool aggUrgent = false;
uint32_t lastAggFrame = 0;

// ==================== SCHEDULE COORDINATOR STATE ====================
bool schedPublished = false;
bool schedN1 = true, schedN2 = true, schedAlert = false;
uint16_t schedVersion = 0;
uint32_t lastScheduleCheck = 0;
#define SCHEDULE_CHECK_INTERVAL_MS 1000
#define MEMBERSHIP_GRACE_MS 20000   // assume N1/N2 present for the first 20 s after boot

// ==================== TIMERS ====================
uint32_t lastSensorRead = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;

// ==================== 5 s SUMMARY LINE ====================
void app_summary() {
  uint32_t cin = mesh.agg.cum_bytes_in, cout = mesh.agg.cum_bytes_out;
  float red_b = cin ? (1.0f - (float)cout / cin) * 100.0f : 0;
  float red_p = mesh.agg.cum_pkts_in ? (1.0f - (float)mesh.agg.cum_pkts_out / mesh.agg.cum_pkts_in) * 100.0f : 0;
  Serial.printf(" AGG   : %u readings in (from N1+N2) → %u aggregate packets out | data reduction %.1f%% bytes, %.1f%% packets | buffered now N1=%d N2=%d\n",
                mesh.agg.cum_pkts_in, mesh.agg.cum_pkts_out, red_b, red_p, mesh.agg.n1_count, mesh.agg.n2_count);
#if N3_HAS_IR_SENSOR
  Serial.printf(" SENSOR: IR obstacle %s | scheduler: schedule v%u (%s)\n", irObstacle ? "DETECTED" : "clear",
                mesh.tdma.sched_version, mesh.tdma.sched_reason);
#else
  Serial.printf(" SCHED : coordinator, schedule v%u (%s)\n", mesh.tdma.sched_version, mesh.tdma.sched_reason);
#endif
}

// ==================== APP CALLBACKS ====================
void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  Serial.printf("[APP] Data for N3: type=%s origin=N%d seq=%d\n",
    msgTypeStr(pkt.msg_type), pkt.origin_id, pkt.seq_num);
}

void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {}

// Buffer one validated, de-duplicated DATA packet from N1/N2 (RX callback context)
bool agg_buffer_data(const wsn_packet_t& pkt) {
  sensor_payload_t sp;
  getPayload(pkt, sp);
  agg_state_t& agg = mesh.agg;
  bool stored = false;
  uint8_t count = 0;
  bool anomaly = false;

  portENTER_CRITICAL(&agg_mux);
  if (pkt.origin_id == 1 && agg.n1_count < AGG_BUFFER_SIZE) {
    uint8_t i = agg.n1_count++;
    agg.n1_ax[i] = sp.val1; agg.n1_ay[i] = sp.val2; agg.n1_az[i] = sp.val3;
    agg.n1_seq[i] = pkt.seq_num; agg.n1_ts[i] = pkt.timestamp_ms;
    count = agg.n1_count; stored = true;
  } else if (pkt.origin_id == 2 && agg.n2_count < AGG_BUFFER_SIZE) {
    uint8_t i = agg.n2_count++;
    agg.n2_analog[i] = sp.val1; agg.n2_digital[i] = (uint8_t)sp.val2;
    agg.n2_seq[i] = pkt.seq_num; agg.n2_ts[i] = pkt.timestamp_ms;
    count = agg.n2_count; stored = true;
  }
  if (stored) {
    agg.bytes_received += wsn_wire_len(pkt);
    agg.cum_bytes_in += wsn_wire_len(pkt);
    agg.cum_pkts_in++;
    if (count >= AGG_BUFFER_SIZE - 2) aggUrgent = true;
  } else {
    agg.overflow_drops++;
  }
  portEXIT_CRITICAL(&agg_mux);

  if (pkt.origin_id == 1) {
    float mag = sqrtf(sp.val1 * sp.val1 + sp.val2 * sp.val2 + sp.val3 * sp.val3);
    if (mag > VIBRATION_ALERT_THRESHOLD) { lastN1AnomalyMs = millis(); aggUrgent = true; anomaly = true; }
  }
  if (stored) {
    LOGV("[AGG] Buffered N%d data: seq=%d (count=%d)%s\n", pkt.origin_id, pkt.seq_num, count,
                  anomaly ? " — N1 ANOMALY" : "");
  } else {
    LOGE("[AGG] Buffer full for N%d — forwarding seq=%d individually\n", pkt.origin_id, pkt.seq_num);
  }
  return stored;
}

// Intermediate hop: N3 INTERCEPTS sensor data from N1/N2 heading to N4 and
// buffers it for aggregation. Contention-test frames are forwarded unchanged.
bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  if (pkt.msg_type != MSG_DATA || pkt.final_dst_id != SINK_ID) return false;
  if (pkt.origin_id != 1 && pkt.origin_id != 2) return false;
  sensor_payload_t sp;
  getPayload(pkt, sp);
  if (strncmp(sp.sensor_type, "colltest", sizeof(sp.sensor_type)) == 0) return false;
  return agg_buffer_data(pkt);   // false (buffer full) → forwarded individually
}

// ==================== AGGREGATION LOGIC ====================
// Snapshot + reset the buffer, compute the aggregate. Returns false if empty.
bool agg_build(agg_payload_t& ap, uint8_t& n1c, uint8_t& n2c) {
  agg_state_t snap;
  portENTER_CRITICAL(&agg_mux);
  snap = mesh.agg;
  mesh.agg.n1_count = 0;
  mesh.agg.n2_count = 0;
  mesh.agg.bytes_received = 0;
  aggUrgent = false;
  portEXIT_CRITICAL(&agg_mux);

  n1c = snap.n1_count;
  n2c = snap.n2_count;
  if (n1c == 0 && n2c == 0) return false;
  memset(&ap, 0, sizeof(ap));

  // N1 vibration: per-axis mean and max magnitude
  if (n1c > 0) {
    float sx = 0, sy = 0, sz = 0, maxm = 0;
    uint32_t oldest = snap.n1_ts[0];
    for (int i = 0; i < n1c; i++) {
      sx += snap.n1_ax[i]; sy += snap.n1_ay[i]; sz += snap.n1_az[i];
      float m = sqrtf(snap.n1_ax[i] * snap.n1_ax[i] + snap.n1_ay[i] * snap.n1_ay[i] + snap.n1_az[i] * snap.n1_az[i]);
      if (m > maxm) maxm = m;
      if ((int32_t)(snap.n1_ts[i] - oldest) < 0) oldest = snap.n1_ts[i];
    }
    ap.n1_accel_mean_x = sx / n1c;
    ap.n1_accel_mean_y = sy / n1c;
    ap.n1_accel_mean_z = sz / n1c;
    ap.n1_accel_max = maxm;
    ap.n1_seq_start = snap.n1_seq[0];
    ap.n1_seq_end = snap.n1_seq[n1c - 1];
    ap.n1_oldest_ts = oldest;
    ap.n1_count = n1c;
  }

  // N2 flame: max and min analog, fire if ANY sample reported fire (DO = 0)
  if (n2c > 0) {
    float amax = snap.n2_analog[0], amin = snap.n2_analog[0];
    uint8_t digital = 1;
    uint32_t oldest = snap.n2_ts[0];
    for (int i = 0; i < n2c; i++) {
      if (snap.n2_analog[i] > amax) amax = snap.n2_analog[i];
      if (snap.n2_analog[i] < amin) amin = snap.n2_analog[i];
      if (snap.n2_digital[i] == 0) digital = 0;
      if ((int32_t)(snap.n2_ts[i] - oldest) < 0) oldest = snap.n2_ts[i];
    }
    ap.n2_flame_analog_max = amax;
    ap.n2_flame_analog_min = amin;
    ap.n2_flame_digital = digital;
    ap.n2_seq_start = snap.n2_seq[0];
    ap.n2_seq_end = snap.n2_seq[n2c - 1];
    ap.n2_oldest_ts = oldest;
    ap.n2_count = n2c;
  }

  ap.n3_ir_obstacle = (uint8_t)irObstacle;
  ap.win_bytes_in = snap.bytes_received;
  ap.win_bytes_out = WSN_HEADER_LEN + sizeof(agg_payload_t);
  return true;
}

void agg_send_window() {
  agg_payload_t ap;
  uint8_t n1c, n2c;
  if (!agg_build(ap, n1c, n2c)) {
    mesh_note_unused_slot("aggregation buffer empty");
    return;
  }
  portENTER_CRITICAL(&agg_mux);
  mesh.agg.cum_bytes_out += ap.win_bytes_out;
  mesh.agg.cum_pkts_out++;
  uint32_t cin = mesh.agg.cum_bytes_in, cout = mesh.agg.cum_bytes_out;
  uint16_t pin = mesh.agg.cum_pkts_in, pout = mesh.agg.cum_pkts_out;
  portEXIT_CRITICAL(&agg_mux);

  float win_red = (ap.win_bytes_in > 0) ? (1.0f - (float)ap.win_bytes_out / ap.win_bytes_in) * 100.0f : 0.0f;
  float cum_red = (cin > 0) ? (1.0f - (float)cout / cin) * 100.0f : 0.0f;
  float pkt_red = (pin > 0) ? (1.0f - (float)pout / pin) * 100.0f : 0.0f;

  LOGV("[AGG] Window: N1 %d pkt(s) [seq %d-%d] mean=(%.2f,%.2f,%.2f) max|a|=%.2f + N2 %d pkt(s) [seq %d-%d] flame max=%.0f min=%.0f DO=%d + N3 IR=%d\n",
                n1c, ap.n1_seq_start, ap.n1_seq_end, ap.n1_accel_mean_x, ap.n1_accel_mean_y, ap.n1_accel_mean_z,
                ap.n1_accel_max, n2c, ap.n2_seq_start, ap.n2_seq_end, ap.n2_flame_analog_max,
                ap.n2_flame_analog_min, ap.n2_flame_digital, ap.n3_ir_obstacle);
  LOGV("[AGG] Data reduction: window %d pkts/%d B in → 1 pkt/%d B out = %.1f%% bytes | cumulative %.1f%% bytes, %.1f%% packets\n",
                n1c + n2c, ap.win_bytes_in, ap.win_bytes_out, win_red, cum_red, pkt_red);

  mesh_own_tx_load(OWN_TX_DATA, mesh_build_agg_packet(ap));
}

// ==================== DYNAMIC SCHEDULE (Cluster Head) ====================
bool nodePresent(uint8_t id) {
  return millis() < MEMBERSHIP_GRACE_MS || mesh.neighbors[id].link_alive || mesh.killed_links[id];
}

void check_schedule_updates() {
  if (!mesh.tdma.synced) return;
  if (millis() - lastScheduleCheck < SCHEDULE_CHECK_INTERVAL_MS) return;
  lastScheduleCheck = millis();

  bool n1 = nodePresent(1);
  bool n2 = nodePresent(2);
  bool alert = n1 && lastN1AnomalyMs != 0 && wsn_age_ms(lastN1AnomalyMs) < VIBRATION_ALERT_HOLD_MS;

  const char* reason = nullptr;
  if (!schedPublished)                 reason = "INIT: default schedule";
  else if (alert && !schedAlert)       reason = "ALERT: N1 extra slot S4";
  else if (!alert && schedAlert)       reason = "NORMAL: alert cleared";
  else if (!n1 && schedN1)             reason = "RELEASE: N1 offline";
  else if (n1 && !schedN1)             reason = "RESTORE: N1 back";
  else if (!n2 && schedN2)             reason = "RELEASE: N2 offline";
  else if (n2 && !schedN2)             reason = "RESTORE: N2 back";
  if (!reason) return;

  schedPublished = true;
  schedN1 = n1; schedN2 = n2; schedAlert = alert;
  schedVersion++;
  schedule_payload_t sp;
  tdma_build_schedule(sp, n1, n2, true, alert, reason, schedVersion, MY_NODE_ID);
  LOGE("[TDMA] *** Schedule update v%u: %s ***\n", schedVersion, reason);
  tdma_publish_schedule(sp);
}

// ==================== SETUP ====================
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 3 (Cluster Head / Aggregator / IR) BOOTING =====");

#if N3_HAS_IR_SENSOR
  pinMode(IR_DO_PIN, INPUT);
  lastRawReading = digitalRead(IR_DO_PIN);
  stableState = lastRawReading;
  lastChangeTime = millis();
  Serial.println("[SENSOR] IR obstacle sensor configured (GPIO 4)");
#else
  Serial.println("[SENSOR] No IR sensor fitted (N3_HAS_IR_SENSOR=0) — N3 works as cluster head/aggregator/router only");
#endif

  ble_init();
  ble_boot_discovery();

  mesh_init();

  Serial.println("===== N3 SETUP COMPLETE =====\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t now = millis();

  // 1. Mesh periodic tasks (sync, HELLO, schedule announcements, forwarding)
  mesh_loop();

  // 2. BLE
  ble_tick();
  ble_serial_poll();   // Serial Monitor: status | ble | cal <id> <m> | setn <n>
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

  // 3. IR sensor with debounce
  if (N3_HAS_IR_SENSOR && now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;
    int raw = digitalRead(IR_DO_PIN);
    if (raw != lastRawReading) {
      lastChangeTime = now;
      lastRawReading = raw;
    }
    if ((now - lastChangeTime) > DEBOUNCE_MS) {
      float prev = irObstacle;
      stableState = lastRawReading;
      irObstacle = (stableState == LOW) ? 1.0f : 0.0f;
      if (irObstacle != prev) LOGE("[SENSOR] IR: %s\n", irObstacle ? "OBSTACLE" : "clear");
    }
  }

  // 4. Aggregation: one AGG per window in my TX slot (S3)
  static uint32_t lastOpportunity = 0xFFFFFFFF;
  if (mesh_take_data_opportunity(lastOpportunity)) {
    {
      uint32_t frame = tdma_current_frame();
      bool window_done = !mesh.tdma.synced || aggUrgent || (frame - lastAggFrame) >= AGG_WINDOW_FRAMES;
      if (mesh_own_tx_busy(OWN_TX_DATA)) {
        // previous AGG still pending → retransmitted below
      } else if (window_done) {
        lastAggFrame = frame;
        agg_send_window();
        if (mesh_own_tx_busy(OWN_TX_DATA)) {
          LOGV("[TDMA] %s: TX opportunity — sending AGG seq=%d via N%d\n",
                        epochStr(tdma_current_epoch()), mesh.own_tx[OWN_TX_DATA].pkt.seq_num,
                        mesh.next_hop_to_sink);
        }
      } else {
        mesh_note_unused_slot("aggregation window still collecting");
      }
    }
  }
  if (mesh_own_tx_busy(OWN_TX_DATA) && mesh_retry_window_open()) {
    mesh_own_tx_service(OWN_TX_DATA);
  }

  // 5. Dynamic schedule (Cluster Head responsibility)
  check_schedule_updates();

  // 6. Compact 5 s status block (full detail: type 'status' or 'log 2')
  wsn_summary_tick();
}
