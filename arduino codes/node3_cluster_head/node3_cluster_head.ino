// ==================== NODE 3: CLUSTER HEAD / AGGREGATOR / IR SENSOR ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Sensor: IR Obstacle Sensor (Digital, GPIO 4)
// Additional roles:
//   - CLUSTER HEAD: computes and broadcasts TDMA schedule
//   - AGGREGATOR: collects N1 + N2 data, fuses into single AGG packet
//   - ROUTER: forwards packets toward the sink (N4)
// Primary route: N3 → N4 (direct)    Alternate: N3 → N2 → N4
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
#define IR_DO_PIN     4       // Digital obstacle sensor (active-low: LOW=obstacle)
#define DEBOUNCE_MS   50
#define SENSOR_READ_INTERVAL_MS 250

// Debounce state
int lastRawReading = HIGH;
int stableState = HIGH;
uint32_t lastChangeTime = 0;

// Latest sensor reading
float irObstacle = 0;  // 1 = obstacle, 0 = clear

// ==================== AGGREGATION STATE ====================
// The aggregation window matches the superframe.
// N3 collects N1 and N2 data during Slots 1 & 2, then sends AGG in Slot 3.
uint32_t lastAggSend = 0;
#define AGG_INTERVAL_MS  SUPERFRAME_MS  // Aggregate once per superframe

// Track whether we need dynamic schedule updates
bool lastN1Alert = false;
bool lastN2Idle = true;
uint32_t lastScheduleUpdate = 0;
#define SCHEDULE_CHECK_INTERVAL_MS 5000

// ==================== TIMERS ====================
uint32_t lastSensorRead = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;

// ==================== APP CALLBACKS ====================
// Called when a packet with final_dst_id == MY_NODE_ID (3) arrives.
// As Cluster Head, N3 receives DATA from N1 and N2 for aggregation.
// These packets have final_dst=4 (sink), NOT final_dst=3.
// So this callback fires only for packets truly addressed to N3 itself.
void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  // Check if this is sensor data from N1 or N2 arriving for aggregation
  // Note: DATA packets going THROUGH N3 toward N4 are handled by the
  // mesh forwarding layer (mesh_handle_data). Here we intercept them
  // at the routing layer to buffer them for aggregation.
  Serial.printf("[APP] Data for N3: type=%s origin=N%d seq=%d\n",
    msgTypeStr(pkt.msg_type), pkt.origin_id, pkt.seq_num);
}

// Called on CTRL commands
void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {
  // N3-specific control handling could go here
}

// Forward declaration (defined below in AGGREGATION LOGIC section)
void agg_buffer_data(const wsn_packet_t& pkt);

// Called when data transits through us (intermediate hop).
// N3 INTERCEPTS sensor data from N1 and N2 heading to N4 — buffers for aggregation.
bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  // Only intercept MSG_DATA from N1 or N2 destined for the sink (N4)
  if (pkt.msg_type == MSG_DATA && pkt.final_dst_id == 4 &&
      (pkt.origin_id == 1 || pkt.origin_id == 2)) {
    agg_buffer_data(pkt);
    return true;  // Consumed — don't forward individually
  }
  return false;  // Other packets: forward normally
}

// ==================== AGGREGATION LOGIC ====================
// Override the mesh forwarding to intercept data destined for N4
// and buffer it for aggregation instead of forwarding immediately.
//
// This is done by hooking into the mesh_handle_data flow.
// When N3 receives MSG_DATA with final_dst=4, instead of forwarding,
// it buffers the data. Then in the AGG slot, it sends a combined packet.

// Buffer incoming data for aggregation
static uint32_t lastN2DataTime = 0;

void agg_buffer_data(const wsn_packet_t& pkt) {
  sensor_payload_t sp;
  getPayload(pkt, sp);
  agg_state_t& agg = mesh.agg;

  if (pkt.origin_id == 1 && agg.n1_count < AGG_BUFFER_SIZE) {
    uint8_t idx = agg.n1_count;
    agg.n1_ax[idx] = sp.val1;
    agg.n1_ay[idx] = sp.val2;
    agg.n1_az[idx] = sp.val3;
    agg.n1_seq[idx] = pkt.seq_num;
    agg.n1_count++;
    agg.bytes_received += pkt.payload_len;
    Serial.printf("[AGG] Buffered N1 data: seq=%d (count=%d)\n", pkt.seq_num, agg.n1_count);
  } else if (pkt.origin_id == 2 && agg.n2_count < AGG_BUFFER_SIZE) {
    uint8_t idx = agg.n2_count;
    agg.n2_analog[idx] = sp.val1;
    agg.n2_digital[idx] = (uint8_t)sp.val2;
    agg.n2_seq[idx] = pkt.seq_num;
    agg.n2_count++;
    agg.bytes_received += pkt.payload_len;
    lastN2DataTime = millis();
    Serial.printf("[AGG] Buffered N2 data: seq=%d (count=%d)\n", pkt.seq_num, agg.n2_count);
  }
}

// Compute and send aggregated packet
void agg_send() {
  agg_state_t& agg = mesh.agg;

  // Need at least some data to aggregate
  if (agg.n1_count == 0 && agg.n2_count == 0) return;

  agg_payload_t ap;
  memset(&ap, 0, sizeof(ap));

  // N1 vibration: compute mean and max magnitude
  if (agg.n1_count > 0) {
    float sum_x = 0, sum_y = 0, sum_z = 0, max_mag = 0;
    for (int i = 0; i < agg.n1_count; i++) {
      sum_x += agg.n1_ax[i];
      sum_y += agg.n1_ay[i];
      sum_z += agg.n1_az[i];
      float mag = sqrtf(agg.n1_ax[i]*agg.n1_ax[i] +
                        agg.n1_ay[i]*agg.n1_ay[i] +
                        agg.n1_az[i]*agg.n1_az[i]);
      if (mag > max_mag) max_mag = mag;
    }
    ap.n1_accel_mean_x = sum_x / agg.n1_count;
    ap.n1_accel_mean_y = sum_y / agg.n1_count;
    ap.n1_accel_mean_z = sum_z / agg.n1_count;
    ap.n1_accel_max = max_mag;
    ap.n1_seq_start = agg.n1_seq[0];
    ap.n1_seq_end = agg.n1_seq[agg.n1_count - 1];
  }

  // N2 flame: max analog, OR of digital
  if (agg.n2_count > 0) {
    float max_analog = 0;
    uint8_t fire_detected = 1;  // 1 = normal
    for (int i = 0; i < agg.n2_count; i++) {
      if (agg.n2_analog[i] > max_analog) max_analog = agg.n2_analog[i];
      if (agg.n2_digital[i] == 0) fire_detected = 0;  // 0 = fire detected
    }
    ap.n2_flame_analog_max = max_analog;
    ap.n2_flame_digital = fire_detected;
    ap.n2_seq_start = agg.n2_seq[0];
    ap.n2_seq_end = agg.n2_seq[agg.n2_count - 1];
  }

  // N3 IR: latest reading
  ap.n3_ir_obstacle = (uint8_t)irObstacle;

  // Data reduction calculation (clamped to non-negative)
  ap.bytes_in = agg.bytes_received;
  ap.bytes_out = sizeof(agg_payload_t);
  ap.reduction_pct = (ap.bytes_in > ap.bytes_out && ap.bytes_in > 0)
    ? (1.0f - (float)ap.bytes_out / (float)ap.bytes_in) * 100.0f
    : 0.0f;

  Serial.printf("[AGG] Sending: N1(%d pkts) + N2(%d pkts) + N3(IR=%d)\n",
                agg.n1_count, agg.n2_count, (int)irObstacle);
  Serial.printf("[AGG] Data reduction: %d bytes in → %d bytes out = %.1f%%\n",
                ap.bytes_in, ap.bytes_out, ap.reduction_pct);

  // Send aggregated data to sink
  mesh_send_agg_data(ap);

  // Reset aggregation buffers
  agg.n1_count = 0;
  agg.n2_count = 0;
  agg.bytes_received = 0;
}

// ==================== DYNAMIC SCHEDULE UPDATES ====================
void check_schedule_updates() {
  if (millis() - lastScheduleUpdate < SCHEDULE_CHECK_INTERVAL_MS) return;
  lastScheduleUpdate = millis();

  agg_state_t& agg = mesh.agg;

  // Check if N1 has vibration anomaly (based on buffered data)
  bool n1_alert = false;
  for (int i = 0; i < agg.n1_count; i++) {
    float mag = sqrtf(agg.n1_ax[i]*agg.n1_ax[i] +
                      agg.n1_ay[i]*agg.n1_ay[i] +
                      agg.n1_az[i]*agg.n1_az[i]);
    if (mag > VIBRATION_ALERT_THRESHOLD) {
      n1_alert = true;
      break;
    }
  }

  // Check if N2 is idle (smooth window: >8 seconds without data)
  bool n2_idle = (millis() - lastN2DataTime > 8000) && mesh.neighbors[2].link_alive;

  // Update schedule if alert/idle status changed, or revert if back to normal
  if (n1_alert != lastN1Alert || n2_idle != lastN2Idle) {
    lastN1Alert = n1_alert;
    lastN2Idle = n2_idle;
    tdma_update_schedule_if_needed(n1_alert, n2_idle, false);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 3 (Cluster Head / Aggregator / IR) BOOTING =====");

  // 1. Initialize IR sensor
  pinMode(IR_DO_PIN, INPUT);
  lastRawReading = digitalRead(IR_DO_PIN);
  stableState = lastRawReading;
  lastChangeTime = millis();
  Serial.println("[SENSOR] IR obstacle sensor configured (GPIO 4)");

  // 2. Initialize BLE
  ble_init();

  // 3. Initialize mesh stack
  mesh_init();

  Serial.println("===== N3 SETUP COMPLETE =====\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t now = millis();

  // 1. Mesh periodic tasks (forwarding, hello, timeouts)
  mesh_loop();

  // 2. BLE scan during sleep epoch
  tdma_epoch_t epoch = tdma_current_epoch();
  if (epoch == EPOCH_SLEEP && now - mesh.last_ble_scan_ms >= 3000) {
    ble_scan();
    mesh.last_ble_scan_ms = now;
  }

  // 3. Send BLE report periodically (using dedicated ctrl sequence)
  if (now - lastBleReport >= 10000) {
    lastBleReport = now;
    ble_report_payload_t rpt = ble_build_report();
    if (rpt.num_neighbors > 0) {
      mesh_send_ble_report(rpt);
    }
  }

  // 4. Read IR sensor with debounce
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;

    int raw = digitalRead(IR_DO_PIN);
    if (raw != lastRawReading) {
      lastChangeTime = now;
      lastRawReading = raw;
    }
    if ((now - lastChangeTime) > DEBOUNCE_MS) {
      stableState = lastRawReading;
    }
    irObstacle = (stableState == LOW) ? 1.0f : 0.0f;
  }

  // 5. Aggregation: send combined packet strictly during assigned Slot 3 (once per superframe)
  static tdma_epoch_t last_agg_slot = EPOCH_SLEEP;
  static uint32_t last_agg_frame = 0xFFFFFFFF;
  uint32_t current_frame = mesh.tdma.frame_number;
  tdma_epoch_t curr_epoch = tdma_current_epoch();

  if (mesh.tdma.synced) {
    if (tdma_is_my_tx_slot()) {
      if (curr_epoch != last_agg_slot || current_frame != last_agg_frame) {
        agg_send();
        last_agg_slot = curr_epoch;
        last_agg_frame = current_frame;
      }
    } else {
      last_agg_slot = curr_epoch;
    }
  } else {
    // Unsynced fallback: send every AGG_INTERVAL_MS (1000ms)
    if (now - lastAggSend >= AGG_INTERVAL_MS) {
      agg_send();
      lastAggSend = now;
    }
  }

  // 6. Dynamic schedule updates (Cluster Head responsibility)
  check_schedule_updates();

  // 7. Print status periodically
  if (now - lastStatusPrint >= 10000) {
    lastStatusPrint = now;
    mesh_print_status();
    ble_print_status();
    Serial.printf("[AGG] Buffer: N1=%d pkts, N2=%d pkts, bytes_in=%d\n",
                  mesh.agg.n1_count, mesh.agg.n2_count, mesh.agg.bytes_received);
  }
}
