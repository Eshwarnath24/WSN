// ==================== WSN COMMON HEADER ====================
// Shared types, constants, packet definitions for all 4 mesh nodes.
// Include this header AFTER defining MY_NODE_ID (1, 2, 3, or 4).
//
// PROTOCOL VERSION 2 — every node must be reflashed together:
//   * header starts with a magic byte and frames are variable length
//     (header + payload_len), so byte-level data reduction is real airtime
//   * ACK carries the acknowledged message type
//   * HELLO carries next hop (poison reverse), synced time and MAC counters
//   * SCHEDULE carries a version and the issuing coordinator
//   * AGG carries exact per-source counts, origin timestamps and
//     cumulative byte/packet counters
// ============================================================
#ifndef WSN_COMMON_H
#define WSN_COMMON_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_idf_version.h>
#include <stddef.h>

#ifndef MY_NODE_ID
  #error "Define MY_NODE_ID (1, 2, 3, or 4) before including wsn_common.h"
#endif

// Printed at boot on every node: check that all 4 boards show the SAME version.
#define WSN_FW_VERSION "WSN-FW 2.3 (Wi-Fi stays on mesh channel, BLE in SLEEP epoch)"

// ==================== NETWORK CONSTANTS ====================
#define NUM_NODES             4
#define SINK_ID               4
#define CLUSTER_HEAD_ID       3
#define BROADCAST_ID          0xFF
#define MAX_PAYLOAD_LEN       64
#define DEFAULT_TTL           4
#define WSN_MAGIC             0xA5    // First byte of every WSN frame (protocol v2)

// ==================== MAC CONSTANTS ====================
#define MAC_MAX_ATTEMPTS      4       // 1 original transmission + 3 retransmissions
#define ACK_TIMEOUT_MS        60      // Per-attempt ACK wait (raise if serial logs show late ACKs)
#define CCA_BUSY_THRESHOLD_MS 4       // Channel busy if a WSN frame was sniffed within this window
#define CW_MIN                4       // Initial contention window (backoff slots)
#define CW_MAX                16      // Maximum contention window (binary exponential backoff)
#define BACKOFF_SLOT_MS       2       // Duration of one backoff slot
#define LINK_FAIL_ACK_TIMEOUTS 4      // Consecutive ACK timeouts to one neighbour → link DEAD (= one exhausted packet)

// ==================== ROUTING CONSTANTS ====================
#define HELLO_INTERVAL_MS     2000
#define HELLO_TIMEOUT_MS      12000   // 6 missed HELLO intervals → link DEAD
#define HELLO_MIN_GAP_MS      300     // Rate limit for triggered HELLOs
#define ROUTE_COST_W_HOP      1.0f
#define ROUTE_COST_W_ETX      2.0f
#define ROUTE_COST_W_RSSI     0.5f
#define ROUTE_CLUSTER_PENALTY 10.0f   // N1/N2 → N4 direct while cluster head N3 has a route
#define ROUTE_LEAF_RELAY_BIAS 0.5f    // Prefer the relay node (N2) over leaf sensor (N1) as relay
#define ROUTE_INFINITY        9999.0f
#define HOPS_INFINITY         255

// ==================== TDMA CONSTANTS ====================
#define SUPERFRAME_MS         800
#define BEACON_START_MS       0
#define BEACON_DURATION_MS    40
#define CAP_START_MS          40
#define CAP_DURATION_MS       120
#define SLOT1_START_MS        160
#define SLOT1_DURATION_MS     120
#define SLOT2_START_MS        280
#define SLOT2_DURATION_MS     120
#define SLOT3_START_MS        400
#define SLOT3_DURATION_MS     150
#define SLOT4_START_MS        550
#define SLOT4_DURATION_MS     120
#define SLEEP_START_MS        670
#define SLEEP_DURATION_MS     130
#define GUARD_MS              5       // Guard interval at the end of each epoch
#define BEACON_STALE_MS       3000    // Beacon older than this → accept sync from neighbour HELLO
#define SYNC_LOST_MS          8000    // No beacon and no synced HELLO → resynchronise (channel scan)
#define SCHED_REBROADCAST_MS  4000    // Coordinator re-announces the CURRENT schedule
#define SCHED_PRIMARY_TIMEOUT_MS 10000 // N4 failover schedule accepted only if N3 silent this long
#define TDMA_LOG_EVERY_FRAMES 10      // Frame summary log period
#ifndef WSN_ENABLE_LIGHT_SLEEP
#define WSN_ENABLE_LIGHT_SLEEP 0      // 1 = esp_light_sleep in SLEEP epoch on leaf sensor N1 (UNVERIFIED on hardware)
#endif

// ==================== BLE CONSTANTS ====================
#define BLE_SCAN_DURATION_MS  110     // Scan burst (100% window); must fit inside the 130 ms SLEEP epoch
#define BLE_SCAN_PERIOD_MS    700     // => a burst in (almost) every superframe
#define BLE_BOOT_DISCOVERY_MS 1500    // Initial BLE discovery before Wi-Fi start
#define BLE_ADV_INTERVAL_MS   50      // continuous advertising while not TDMA-synced (boot discovery)
#define BLE_ADV_BURST_INTERVAL_MS 25  // synced: advertise only in the SLEEP epoch, 25–30 ms => ~4 adverts per scan
#define BLE_TX_POWER_1M       (-59)   // Calibrated RSSI at 1 m (calibrate per environment)
#define BLE_PATH_LOSS_N       2.2f    // Path loss exponent (indoor)
#define BLE_EWMA_ALPHA        0.3f
#define BLE_NEIGHBOR_EXPIRY_MS 30000  // Forget BLE neighbours not seen for this long
#define BLE_MIN_DIST_M        0.2f    // Distance clamp (RSSI-derived ranges)
#define BLE_MAX_DIST_M        15.0f
#define BLE_AREA_MARGIN_M     2.0f    // Localization estimate kept within reference area ± margin
#define BLE_REPORT_PERIOD_MS  10000

// ==================== SENSOR ALERT THRESHOLDS ====================
#ifndef VIBRATION_ALERT_THRESHOLD
#define VIBRATION_ALERT_THRESHOLD 15.0f   // m/s^2 total acceleration magnitude
#endif
#define VIBRATION_ALERT_HOLD_MS   10000   // Alert schedule held this long after last anomaly

// ==================== NODE ROLES ====================
enum node_role_t : uint8_t {
  ROLE_SENSOR       = 0,
  ROLE_RELAY        = 1,
  ROLE_CLUSTER_HEAD = 2,
  ROLE_SINK         = 3
};

// ==================== MESSAGE TYPES ====================
enum msg_type_t : uint8_t {
  MSG_HELLO        = 0x01,  // Broadcast: periodic neighbour/route announcement (CAP)
  MSG_BEACON       = 0x02,  // Broadcast: time-sync beacon from N4 (BEACON epoch)
  MSG_SCHEDULE     = 0x03,  // Broadcast: TDMA slot table from N3 (or N4 in failover) (CAP)
  MSG_DATA         = 0x10,  // Unicast:   sensor reading, hop-by-hop
  MSG_AGG          = 0x11,  // Unicast:   aggregated data from N3
  MSG_ACK          = 0x20,  // Unicast:   application-level acknowledgement
  MSG_ROUTE_UPDATE = 0x30,  // Broadcast: link state change announcement (CAP)
  MSG_BLE_REPORT   = 0x40,  // Unicast:   BLE distance table (to sink, multi-hop)
  MSG_CTRL         = 0xF0   // Broadcast: dashboard/serial control command from N4
};

// ==================== CONTROL COMMANDS ====================
enum ctrl_cmd_t : uint8_t {
  CTRL_COLLISION_TEST = 0x01,   // N1+N2 transmit at the start of the next CAP (contention demo)
  CTRL_KILL_LINK      = 0x02,   // Software link kill (param1=nodeA, param2=nodeB) — failure INJECTION
  CTRL_RESTORE_LINKS  = 0x03    // Restore all software-killed links
};

// ==================== TDMA SLOT TYPES ====================
enum slot_type_t : uint8_t {
  SLOT_TX    = 0,
  SLOT_RX    = 1,
  SLOT_RELAY = 2,
  SLOT_SLEEP = 3   // Unused / released slot (owner node_id = 0)
};

// ==================== TDMA EPOCH IDS ====================
enum tdma_epoch_t : uint8_t {
  EPOCH_BEACON = 0,
  EPOCH_CAP    = 1,
  EPOCH_SLOT1  = 2,
  EPOCH_SLOT2  = 3,
  EPOCH_SLOT3  = 4,
  EPOCH_SLOT4  = 5,
  EPOCH_SLEEP  = 6
};

// ==================== MAC RESULT ====================
enum mac_result_t : uint8_t {
  MAC_OK       = 0,   // Broadcast sent, or unicast ACKed
  MAC_DEFERRED = 1,   // Not enough time left in this slot/CAP — keep packet, retry later
  MAC_DROPPED  = 2,   // All attempts used without ACK
  MAC_NO_ROUTE = 3    // No next hop
};

// ==================== MAC ADDRESS TABLE ====================
// Initial peer MACs. Real MACs are AUTO-LEARNED from received frames, so the
// placeholders only matter until the first HELLO/BEACON is heard.
// To find a board's MAC: upload a sketch with Serial.println(WiFi.macAddress());
const uint8_t NODE_MAC[NUM_NODES + 1][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Index 0: unused placeholder
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01},  // N1: placeholder (auto-learned at runtime)
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02},  // N2: placeholder (auto-learned at runtime)
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03},  // N3: placeholder (auto-learned at runtime)
  {0x8C, 0x94, 0xDF, 0x7C, 0x3F, 0x58},  // N4: from existing code
};

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ==================== ROLE ASSIGNMENT TABLE ====================
const node_role_t NODE_ROLES[NUM_NODES + 1] = {
  ROLE_SENSOR,         // Index 0: unused
  ROLE_SENSOR,         // N1: Vibration sensor
  ROLE_RELAY,          // N2: Flame sensor + relay
  ROLE_CLUSTER_HEAD,   // N3: IR sensor + aggregator/scheduler
  ROLE_SINK            // N4: Gateway/sink
};

// ==================== PACKET STRUCTURE ====================
#pragma pack(push, 1)
typedef struct {
  // --- Header (19 bytes) ---
  uint8_t  magic;            // WSN_MAGIC
  uint8_t  msg_type;         // msg_type_t
  uint8_t  src_id;           // Immediate transmitter  (1–4)
  uint8_t  dst_id;           // Immediate receiver     (1–4, or 0xFF = broadcast)
  uint8_t  origin_id;        // Original data producer (1–4)
  uint8_t  final_dst_id;     // Ultimate destination   (1–4)
  uint16_t seq_num;          // Per-origin sequence counter
  uint8_t  hop_count;        // Incremented at each forwarding node
  uint8_t  ttl;              // Decremented at each hop; drop at 0
  uint32_t timestamp_ms;     // Origin synced time when packet was first created
  uint8_t  route_path[4];    // Hop trace: path[0]=origin, path[hop_count]=last forwarder
  uint8_t  payload_len;      // Bytes used in payload[]
  // --- Payload ---
  uint8_t  payload[MAX_PAYLOAD_LEN];
} wsn_packet_t;
#pragma pack(pop)

#define WSN_HEADER_LEN  ((int)offsetof(wsn_packet_t, payload))
inline int wsn_wire_len(const wsn_packet_t& p) { return WSN_HEADER_LEN + p.payload_len; }

// ==================== PAYLOAD SUB-STRUCTURES ====================
#pragma pack(push, 1)

// Sensor data payload (used in MSG_DATA)
typedef struct {
  float val1;
  float val2;
  float val3;
  char  sensor_type[10];    // "vibration", "flame", "ir", "colltest"
} sensor_payload_t;          // 22 bytes

// Aggregated data payload (used in MSG_AGG)
typedef struct {
  float    n1_accel_mean_x;
  float    n1_accel_mean_y;
  float    n1_accel_mean_z;
  float    n1_accel_max;      // max |a| in window
  float    n2_flame_analog_max;
  float    n2_flame_analog_min;
  uint8_t  n2_flame_digital;  // 0 if ANY sample in window reported fire (DO active-low)
  uint8_t  n3_ir_obstacle;    // Latest IR reading at N3 (1 = obstacle)
  uint8_t  n1_count;          // Unique N1 readings in this window
  uint8_t  n2_count;          // Unique N2 readings in this window
  uint16_t n1_seq_start;
  uint16_t n1_seq_end;
  uint16_t n2_seq_start;
  uint16_t n2_seq_end;
  uint32_t n1_oldest_ts;      // Origin timestamp of oldest N1 reading (end-to-end delay)
  uint32_t n2_oldest_ts;      // Origin timestamp of oldest N2 reading
  uint16_t win_bytes_in;      // Wire bytes of the DATA frames aggregated in this window
  uint16_t win_bytes_out;     // Wire bytes of this AGG frame (header + payload)
} agg_payload_t;              // 48 bytes

// Time-sync beacon payload (used in MSG_BEACON)
typedef struct {
  uint32_t frame_number;
  uint32_t timestamp_ms;      // N4's millis() at beacon TX
} beacon_payload_t;           // 8 bytes

// TDMA schedule payload (used in MSG_SCHEDULE)
typedef struct {
  uint16_t version;           // Incremented by the coordinator on every change
  uint8_t  issuer;            // 3 = cluster head (primary), 4 = sink (failover)
  uint8_t  slot_count;
  struct {
    uint8_t node_id;          // 0 = unused/released slot
    uint8_t slot_index;       // EPOCH_SLOT1 .. EPOCH_SLOT4
    uint8_t slot_type;        // slot_type_t
  } slots[8];
  char update_reason[24];     // Human-readable reason for schedule change
} schedule_payload_t;         // 52 bytes

// HELLO payload (used in MSG_HELLO)
typedef struct {
  uint8_t  node_id;
  uint8_t  hops_to_sink;
  uint8_t  next_hop;          // Sender's current next hop (poison reverse + dashboard topology)
  float    ack_success_rate;  // 0.0–1.0 toward sender's next hop
  int8_t   wifi_rssi;         // Sender's RSSI from its next hop
  uint16_t node_retries;      // Cumulative retransmissions by sender
  uint16_t cca_busy;          // Cumulative CCA-busy detections
  uint16_t backoffs;          // Cumulative random backoffs
  uint16_t ack_timeouts;      // Cumulative ACK timeouts
  uint16_t drops;             // Cumulative MAC drops
  uint8_t  synced;            // 1 if sender is TDMA-synchronised
  uint32_t synced_time_ms;    // Sender's synced clock at TX (multi-hop time sync)
  uint8_t  duty_pct;          // Sender's radio duty cycle (active epochs), 0–100
} hello_payload_t;            // 24 bytes

// ACK payload (used in MSG_ACK)
typedef struct {
  uint16_t acked_seq_num;     // Sequence number being acknowledged
  uint8_t  acked_origin_id;   // Origin of the acknowledged packet
  uint8_t  acked_type;        // msg_type of the acknowledged packet
} ack_payload_t;              // 4 bytes

// Route update payload (used in MSG_ROUTE_UPDATE)
typedef struct {
  uint8_t node_a;             // One end of the failed/restored link
  uint8_t node_b;             // Other end
  uint8_t link_up;            // 1 = restored, 0 = failed
} route_update_payload_t;     // 3 bytes

// BLE report payload (used in MSG_BLE_REPORT)
typedef struct {
  uint8_t reporter_id;
  uint8_t nearest_node_id;    // Reporter's local nearest neighbour
  uint8_t num_neighbors;
  struct {
    uint8_t node_id;
    int8_t  rssi;             // EWMA-filtered RSSI (dBm, rounded)
    float   distance_m;
  } entries[3];               // Max 3 other nodes
  float est_x, est_y;         // Reporter's local estimate (0 if not available)
  float true_x, true_y;       // Reference position (tape-measured, pre-configured)
  float error_m;              // Reporter's local localization error (-1 if not available)
} ble_report_payload_t;       // 41 bytes

// Control command payload (used in MSG_CTRL)
typedef struct {
  uint8_t cmd;                // ctrl_cmd_t
  uint8_t param1;
  uint8_t param2;
} ctrl_payload_t;             // 3 bytes

#pragma pack(pop)

// ==================== NEIGHBOR / ROUTING STRUCTURES ====================
struct route_neighbor_t {
  uint8_t  node_id;
  int8_t   wifi_rssi;
  float    ack_success_rate;
  uint8_t  hops_to_sink;
  uint8_t  next_hop;          // Neighbour's advertised next hop (0 = unknown)
  uint32_t last_hello_ms;
  bool     link_alive;
  uint16_t reported_retries;  // Neighbour's cumulative retransmissions from HELLO
  uint16_t reported_cca_busy;
  uint16_t reported_backoffs;
  uint16_t reported_ack_timeouts;
  uint16_t reported_drops;
  uint8_t  reported_duty_pct;
  // Running ACK stats
  uint16_t ack_sent;
  uint16_t ack_received;
  uint8_t  consec_ack_fails;  // Consecutive ACK timeouts (attempt level)
};

// ==================== MAC STATE ====================
struct mac_stats_t {
  uint32_t cca_busy_count;
  uint32_t total_backoffs;
  uint32_t total_tx_attempts;   // Every unicast transmission (original + retransmissions)
  uint32_t total_retries;       // Retransmissions only (attempts beyond the first)
  uint32_t total_drops;
  uint32_t total_acks_ok;
  uint32_t total_acks_timeout;
  uint32_t total_deferrals;     // Packets carried over to the next slot/CAP
  uint32_t total_reroutes;      // Packets re-addressed to an alternate next hop
  uint32_t dup_rx;              // Duplicate frames received (re-ACKed, not re-processed)
  uint32_t malformed_rx;        // Frames rejected by validation
  volatile uint32_t ll_fail;    // 802.11 link-layer send failures (no MAC-level ACK from the peer radio)
};

// ==================== TDMA STATE ====================
struct tdma_state_t {
  int32_t      clock_offset;          // sink time - local millis()
  uint32_t     frame_number;          // Derived from synced time
  bool         synced;
  bool         synced_from_beacon;    // false = adopted from a neighbour HELLO
  uint32_t     last_beacon_rx_ms;
  uint32_t     last_sync_ms;          // Last beacon OR synced HELLO used for sync
  // Current schedule
  uint8_t      my_tx_slot;
  uint8_t      my_relay_slot;
  uint8_t      my_tx_slots_mask;      // Bitmask of epochs where this node sends its own data
  uint8_t      my_relay_slots_mask;   // Bitmask of epochs where this node forwards queued data
  uint8_t      slot_owner[7];         // Owner node per epoch (0 = unused)
  bool         schedule_received;
  uint16_t     sched_version;
  uint8_t      sched_issuer;
  uint32_t     last_sched_rx_ms;
  uint32_t     last_primary_sched_ms; // Last schedule heard from N3
  char         sched_reason[24];
  // Duty cycle / sleep tracking
  uint32_t     active_time_ms;
  uint32_t     sleep_time_ms;
  uint32_t     last_epoch_switch_ms;
  tdma_epoch_t last_epoch;
  bool         last_epoch_active;
  uint32_t     unused_slots;          // Count of own TX slots skipped for lack of data
  uint32_t     last_logged_frame;
};

// ==================== BLE STATE ====================
struct ble_neighbor_t {
  uint8_t  node_id;
  int8_t   rssi_raw;
  float    rssi_filtered;
  float    distance_m;
  uint32_t last_seen_ms;
  bool     discovered;
};

// ==================== FORWARDING QUEUE (DECOUPLING RX FROM TX) ====================
#define FWD_QUEUE_SIZE 8
struct fwd_entry_t {
  wsn_packet_t pkt;
  uint8_t      attempts;
};
struct fwd_queue_t {
  fwd_entry_t queue[FWD_QUEUE_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
};

// ==================== OWN-PACKET TX BUFFER ====================
// Packets originated by this node that are waiting for (re)transmission.
struct own_tx_t {
  bool         active;
  wsn_packet_t pkt;
  uint8_t      attempts;
  uint32_t     defer_key;   // TX opportunity in which the packet was deferred
};
#define OWN_TX_DATA 0   // DATA (N1/N2) or AGG (N3)
#define OWN_TX_BLE  1   // BLE report

// ==================== DUPLICATE CACHE ====================
#define DUP_CACHE_SIZE 32
#define DUP_CACHE_TTL_MS 10000
struct dup_entry_t {
  uint8_t  type;
  uint8_t  origin;
  uint16_t seq;
  uint32_t t_ms;
};

// ==================== AGGREGATION STATE (N3 only) ====================
#define AGG_BUFFER_SIZE  8
#ifndef AGG_WINDOW_FRAMES
#define AGG_WINDOW_FRAMES 2     // N3 sends one AGG every 2 superframes (1.6 s); S3 is idle in between
#endif
struct agg_state_t {
  // N1 vibration buffer
  float    n1_ax[AGG_BUFFER_SIZE];
  float    n1_ay[AGG_BUFFER_SIZE];
  float    n1_az[AGG_BUFFER_SIZE];
  uint16_t n1_seq[AGG_BUFFER_SIZE];
  uint32_t n1_ts[AGG_BUFFER_SIZE];
  uint8_t  n1_count;
  // N2 flame buffer
  float    n2_analog[AGG_BUFFER_SIZE];
  uint8_t  n2_digital[AGG_BUFFER_SIZE];
  uint16_t n2_seq[AGG_BUFFER_SIZE];
  uint32_t n2_ts[AGG_BUFFER_SIZE];
  uint8_t  n2_count;
  // Byte tracking (wire bytes of aggregated DATA frames)
  uint16_t bytes_received;
  uint32_t cum_bytes_in;
  uint32_t cum_bytes_out;
  uint16_t cum_pkts_in;
  uint16_t cum_pkts_out;
  uint32_t overflow_drops;
};

// ==================== GLOBAL MESH STATE ====================
struct mesh_state_t {
  // Identity
  uint8_t      my_id;
  node_role_t  my_role;

  // Neighbour / routing table (index 0 unused, 1..4 for nodes)
  route_neighbor_t neighbors[NUM_NODES + 1];
  uint8_t          next_hop_to_sink;      // Best next hop toward N4
  float            next_hop_cost;
  uint8_t          peer_macs[NUM_NODES + 1][6]; // Dynamically learned peer MACs
  uint32_t         last_heard_ms[NUM_NODES + 1]; // Any frame from node (src)

  // MAC
  mac_stats_t      mac_stats;
  volatile uint32_t last_rx_time_ms;       // For software CCA
  uint8_t          current_cw;             // Current contention window

  // Volatile ACK state (shared between RX callback and main loop)
  volatile bool     ack_pending;
  volatile uint16_t ack_expected_seq;
  volatile uint8_t  ack_expected_origin;
  volatile uint8_t  ack_expected_type;
  volatile uint8_t  ack_expected_from;
  volatile bool     ack_received;

  // TDMA
  tdma_state_t     tdma;

  // Non-blocking forwarding queue
  fwd_queue_t      fwd_queue;

  // Own packets waiting for (re)transmission
  own_tx_t         own_tx[2];

  // Duplicate suppression
  dup_entry_t      dup_cache[DUP_CACHE_SIZE];
  uint8_t          dup_next;

  // BLE (index 0 unused, 1..4)
  ble_neighbor_t   ble[NUM_NODES + 1];

  // Aggregation (N3 only)
  agg_state_t      agg;

  // Software link kills (for demo failure INJECTION — not a physical failure)
  bool             killed_links[NUM_NODES + 1];

  // Sequence counters
  uint16_t         data_seq_counter;      // DATA / AGG / colltest (sink PDR is computed on this)
  uint16_t         ctrl_seq_counter;      // CTRL / BLE report / route update

  // Performance counters
  uint32_t         pkts_sent;
  uint32_t         pkts_forwarded;
  uint32_t         pkts_received_at_sink;  // Only relevant for N4
  uint32_t         dup_rx_by_origin[NUM_NODES + 1];

  // Timers / flags
  uint32_t         last_hello_ms;
  uint32_t         last_ble_scan_ms;
  volatile bool    hello_triggered;
  volatile bool    route_update_pending;
  route_update_payload_t route_update_payload;
  volatile bool    sched_announce_pending;
  schedule_payload_t sched_current;       // Coordinator's current schedule (N3 / N4 failover)
  bool             sched_is_coordinator;
  uint32_t         last_sched_announce_ms;
  schedule_payload_t last_primary_sched;  // N4: latest N3 schedule, relayed for nodes out of N3's range
  uint32_t         last_primary_rx_ms;
  uint32_t         last_sched_relay_ms;

  // Collision test flag
  volatile bool    collision_test_trigger;
};

// ==================== UTILITY FUNCTIONS ====================

inline const char* nodeIdStr(uint8_t id) {
  static const char* names[] = {"N0", "N1", "N2", "N3", "N4"};
  if (id == BROADCAST_ID) return "BCAST";
  if (id <= NUM_NODES) return names[id];
  return "N?";
}

inline const char* msgTypeStr(uint8_t type) {
  switch (type) {
    case MSG_HELLO:        return "HELLO";
    case MSG_BEACON:       return "BEACON";
    case MSG_SCHEDULE:     return "SCHEDULE";
    case MSG_DATA:         return "DATA";
    case MSG_AGG:          return "AGG";
    case MSG_ACK:          return "ACK";
    case MSG_ROUTE_UPDATE: return "ROUTE_UPD";
    case MSG_BLE_REPORT:   return "BLE_RPT";
    case MSG_CTRL:         return "CTRL";
    default:               return "???";
  }
}

inline const char* roleStr(node_role_t role) {
  switch (role) {
    case ROLE_SENSOR:       return "Sensor";
    case ROLE_RELAY:        return "Relay";
    case ROLE_CLUSTER_HEAD: return "ClusterHead";
    case ROLE_SINK:         return "Sink";
    default:                return "Unknown";
  }
}

inline const char* epochStr(uint8_t e) {
  static const char* names[] = {"BEACON", "CAP", "S1", "S2", "S3", "S4", "SLEEP"};
  return (e <= EPOCH_SLEEP) ? names[e] : "?";
}

// ==================== TIME HELPERS ====================
// Age of a timestamp that ANOTHER TASK may have just written (Wi-Fi / BLE
// callbacks). With "now" sampled earlier, plain (now - t) wraps to ~4.29e9 ms
// when t is a few ms in the future: on hardware this caused false "sync lost",
// false BLE "expired", false link timeouts and nodes flagged INACTIVE.
inline uint32_t wsn_age_ms(uint32_t t) {
  int32_t d = (int32_t)(millis() - t);
  return (d > 0) ? (uint32_t)d : 0;
}

// ==================== LOGGING ====================
extern uint8_t  wsn_log_level;
extern uint32_t wsn_log_force_until;
inline bool wsn_verbose() { return wsn_log_level >= 2 || (int32_t)(wsn_log_force_until - millis()) > 0; }
#define LOGV(...) do { if (wsn_verbose()) Serial.printf(__VA_ARGS__); } while (0)                       // per-packet trace
#define LOGE(...) do { if (wsn_log_level >= 1 || wsn_verbose()) Serial.printf(__VA_ARGS__); } while (0) // events

inline void logPacket(const char* dir, const wsn_packet_t& p) {
  if (!wsn_verbose()) return;
  Serial.printf("[%s] type=%-9s src=N%d dst=%-5s origin=N%d final=%s seq=%d hops=%d ttl=%d len=%d\n",
    dir, msgTypeStr(p.msg_type),
    p.src_id, nodeIdStr(p.dst_id),
    p.origin_id, nodeIdStr(p.final_dst_id),
    p.seq_num, p.hop_count, p.ttl, wsn_wire_len(p));
}

extern mesh_state_t mesh;

// Initialize a packet with standard header fields
inline void initPacket(wsn_packet_t& pkt, uint8_t type, uint8_t dst,
                       uint8_t origin, uint8_t final_dst, uint16_t seq) {
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic        = WSN_MAGIC;
  pkt.msg_type     = type;
  pkt.src_id       = MY_NODE_ID;
  pkt.dst_id       = dst;
  pkt.origin_id    = origin;
  pkt.final_dst_id = final_dst;
  pkt.seq_num      = seq;
  pkt.hop_count    = 0;
  pkt.ttl          = DEFAULT_TTL;
  pkt.timestamp_ms = (mesh.tdma.synced) ? (millis() + mesh.tdma.clock_offset) : millis();
  pkt.route_path[0] = origin;
}

// Copy a typed payload into a packet's payload buffer
template <typename T>
inline void setPayload(wsn_packet_t& pkt, const T& data) {
  static_assert(sizeof(T) <= MAX_PAYLOAD_LEN, "Payload too large");
  memcpy(pkt.payload, &data, sizeof(T));
  pkt.payload_len = sizeof(T);
}

// Extract a typed payload from a packet's payload buffer
template <typename T>
inline void getPayload(const wsn_packet_t& pkt, T& data) {
  memcpy(&data, pkt.payload, sizeof(T));
}

// Minimum payload length per message type (validation)
inline int wsn_min_payload_len(uint8_t type) {
  switch (type) {
    case MSG_HELLO:        return sizeof(hello_payload_t);
    case MSG_BEACON:       return sizeof(beacon_payload_t);
    case MSG_SCHEDULE:     return sizeof(schedule_payload_t);
    case MSG_DATA:         return sizeof(sensor_payload_t);
    case MSG_AGG:          return sizeof(agg_payload_t);
    case MSG_ACK:          return sizeof(ack_payload_t);
    case MSG_ROUTE_UPDATE: return sizeof(route_update_payload_t);
    case MSG_BLE_REPORT:   return sizeof(ble_report_payload_t);
    case MSG_CTRL:         return sizeof(ctrl_payload_t);
    default:               return -1;
  }
}

#endif // WSN_COMMON_H
