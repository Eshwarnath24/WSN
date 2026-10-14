// ==================== WSN COMMON HEADER ====================
// Shared types, constants, packet definitions for all 4 mesh nodes.
// Include this header AFTER defining MY_NODE_ID (1, 2, 3, or 4).
// ============================================================
#ifndef WSN_COMMON_H
#define WSN_COMMON_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_idf_version.h>

#ifndef MY_NODE_ID
  #error "Define MY_NODE_ID (1, 2, 3, or 4) before including wsn_common.h"
#endif

// ==================== NETWORK CONSTANTS ====================
#define NUM_NODES             4
#define BROADCAST_ID          0xFF
#define MAX_PAYLOAD_LEN       64
#define DEFAULT_TTL           4

// ==================== MAC CONSTANTS ====================
#define MAX_RETRIES           3
#define ACK_TIMEOUT_MS        80      // Increased to 80ms to avoid false timeouts (actual RTT is ~48ms)
#define CCA_BUSY_THRESHOLD_MS 3       // Channel busy if frame seen within this window
#define CW_MIN                4
#define CW_MAX                64
#define BACKOFF_SLOT_MS       5

// ==================== ROUTING CONSTANTS ====================
#define HELLO_INTERVAL_MS     2000
#define HELLO_TIMEOUT_MS      12000   // 12s timeout (6 hello intervals) prevents premature link death
#define LINK_FAIL_THRESHOLD   3       // 3 consecutive DROPPED packets (not retries) → link dead
#define ROUTE_COST_W_HOP      1.0f
#define ROUTE_COST_W_ETX      2.0f
#define ROUTE_COST_W_RSSI     0.5f

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
#define GUARD_MS              5       // Guard interval within each slot

// ==================== BLE CONSTANTS ====================
#define BLE_SCAN_DURATION_MS  200     // Scan burst during sleep epoch
#define BLE_ADV_INTERVAL_MS   100
#define BLE_TX_POWER_1M       (-59)   // Calibrated RSSI at 1 m (adjust per environment)
#define BLE_PATH_LOSS_N       2.2f    // Path loss exponent (indoor)
#define BLE_EWMA_ALPHA        0.3f

// ==================== SENSOR ALERT THRESHOLDS ====================
#ifndef VIBRATION_ALERT_THRESHOLD
#define VIBRATION_ALERT_THRESHOLD 15.0f   // m/s^2 total acceleration magnitude
#endif

// ==================== NODE ROLES ====================
enum node_role_t : uint8_t {
  ROLE_SENSOR       = 0,
  ROLE_RELAY        = 1,
  ROLE_CLUSTER_HEAD = 2,
  ROLE_SINK         = 3
};

// ==================== MESSAGE TYPES ====================
enum msg_type_t : uint8_t {
  MSG_HELLO        = 0x01,  // Broadcast: periodic mesh neighbor announcement
  MSG_BEACON       = 0x02,  // Broadcast: time-sync beacon from N4
  MSG_SCHEDULE     = 0x03,  // Broadcast: TDMA slot table from N3 (Cluster Head)
  MSG_DATA         = 0x10,  // Unicast:   sensor reading, hop-by-hop
  MSG_AGG          = 0x11,  // Unicast:   aggregated data from N3
  MSG_ACK          = 0x20,  // Unicast:   application-level acknowledgement
  MSG_ROUTE_UPDATE = 0x30,  // Broadcast: link state change announcement
  MSG_BLE_REPORT   = 0x40,  // Unicast:   BLE distance table (to sink)
  MSG_CTRL         = 0xF0   // Broadcast: dashboard control command
};

// ==================== CONTROL COMMANDS ====================
enum ctrl_cmd_t : uint8_t {
  CTRL_COLLISION_TEST = 0x01,   // Force N1+N2 simultaneous TX in CAP
  CTRL_KILL_LINK      = 0x02,   // Software link kill (param1=nodeA, param2=nodeB)
  CTRL_RESTORE_LINKS  = 0x03    // Restore all software-killed links
};

// ==================== TDMA SLOT TYPES ====================
enum slot_type_t : uint8_t {
  SLOT_TX    = 0,
  SLOT_RX    = 1,
  SLOT_RELAY = 2,
  SLOT_SLEEP = 3
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

// ==================== MAC ADDRESS TABLE ====================
// *** IMPORTANT: Replace these with YOUR actual ESP32 MAC addresses! ***
// To find: upload a sketch with: Serial.println(WiFi.macAddress());
const uint8_t NODE_MAC[NUM_NODES + 1][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Index 0: unused placeholder
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01},  // N1: *** REPLACE ***
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02},  // N2: *** REPLACE ***
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03},  // N3: *** REPLACE ***
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
  // --- Header (18 bytes) ---
  uint8_t  msg_type;         // msg_type_t
  uint8_t  src_id;           // Immediate transmitter  (1–4)
  uint8_t  dst_id;           // Immediate receiver     (1–4, or 0xFF = broadcast)
  uint8_t  origin_id;        // Original data producer (1–4)
  uint8_t  final_dst_id;     // Ultimate destination   (1–4)
  uint16_t seq_num;          // Per-origin sequence counter
  uint8_t  hop_count;        // Incremented at each forwarding node
  uint8_t  ttl;              // Decremented at each hop; drop at 0
  uint32_t timestamp_ms;     // Origin millis() when packet was first created
  uint8_t  route_path[4];    // Hop trace: path[0]=origin, path[hop_count]=current
  uint8_t  payload_len;      // Bytes used in payload[]
  // --- Payload ---
  uint8_t  payload[MAX_PAYLOAD_LEN];
} wsn_packet_t;
#pragma pack(pop)

// ==================== PAYLOAD SUB-STRUCTURES ====================
#pragma pack(push, 1)

// Sensor data payload (used in MSG_DATA)
typedef struct {
  float val1;
  float val2;
  float val3;
  char  sensor_type[10];    // "vibration", "flame", "ir"
} sensor_payload_t;          // 22 bytes

// Aggregated data payload (used in MSG_AGG)
typedef struct {
  float    n1_accel_mean_x;
  float    n1_accel_mean_y;
  float    n1_accel_mean_z;
  float    n1_accel_max;      // max magnitude
  float    n2_flame_analog_max;
  uint8_t  n2_flame_digital;  // OR of all samples in window
  uint8_t  n3_ir_obstacle;    // Latest IR reading
  uint16_t n1_seq_start;
  uint16_t n1_seq_end;
  uint16_t n2_seq_start;
  uint16_t n2_seq_end;
  uint16_t bytes_in;          // Total raw bytes received
  uint16_t bytes_out;         // Size of this AGG packet
  float    reduction_pct;     // (1 - out/in) * 100
} agg_payload_t;              // 38 bytes

// Time-sync beacon payload (used in MSG_BEACON)
typedef struct {
  uint32_t frame_number;
  uint32_t timestamp_ms;      // N4's millis() at beacon TX
} beacon_payload_t;           // 8 bytes

// TDMA schedule payload (used in MSG_SCHEDULE)
typedef struct {
  uint8_t slot_count;
  struct {
    uint8_t node_id;
    uint8_t slot_index;       // EPOCH_SLOT1 .. EPOCH_SLOT4
    uint8_t slot_type;        // slot_type_t
  } slots[8];
  char update_reason[24];     // Human-readable reason for schedule change
} schedule_payload_t;         // 50 bytes

// HELLO payload (used in MSG_HELLO)
typedef struct {
  uint8_t  node_id;
  uint8_t  hops_to_sink;
  float    ack_success_rate;  // 0.0–1.0
  int8_t   wifi_rssi;        // Sender's best link RSSI
  uint16_t node_retries;      // Cumulative retries by sender
} hello_payload_t;            // 10 bytes

// ACK payload (used in MSG_ACK)
typedef struct {
  uint16_t acked_seq_num;     // Sequence number being acknowledged
  uint8_t  acked_origin_id;   // Origin of the acknowledged packet
} ack_payload_t;              // 3 bytes

// Route update payload (used in MSG_ROUTE_UPDATE)
typedef struct {
  uint8_t node_a;             // One end of the failed/restored link
  uint8_t node_b;             // Other end
  bool    link_up;            // true = restored, false = failed
} route_update_payload_t;     // 3 bytes

// BLE report payload (used in MSG_BLE_REPORT)
typedef struct {
  uint8_t reporter_id;
  uint8_t nearest_node_id;    // Reporter's local nearest neighbor
  uint8_t num_neighbors;
  struct {
    uint8_t node_id;
    int8_t  rssi;
    float   distance_m;
  } entries[3];               // Max 3 other nodes
  float est_x, est_y;        // Estimated position (filled by sink)
  float true_x, true_y;      // True position (pre-configured)
  float error_m;              // Localization error
} ble_report_payload_t;       // 44 bytes

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
  uint32_t last_hello_ms;
  bool     link_alive;
  uint16_t reported_retries;  // Sender's cumulative retries from HELLO
  // Running ACK stats for EWMA
  uint16_t ack_sent;
  uint16_t ack_received;
  uint8_t  consec_ack_fails;
};

// ==================== MAC STATE ====================
struct mac_stats_t {
  uint32_t cca_busy_count;
  uint32_t total_backoffs;
  uint32_t total_retries;
  uint32_t total_drops;
  uint32_t total_acks_ok;
  uint32_t total_acks_timeout;
};

// ==================== TDMA STATE ====================
struct tdma_state_t {
  int32_t      clock_offset;          // beacon.ts - local millis() at rx
  uint32_t     frame_number;
  bool         synced;                // Have we received at least one beacon?
  uint32_t     last_beacon_rx_ms;
  // Current schedule (parsed from MSG_SCHEDULE)
  uint8_t      my_tx_slot;            // Primary slot (e.g. EPOCH_SLOT1)
  uint8_t      my_relay_slot;         // Relay slot (e.g. EPOCH_SLOT4)
  uint8_t      my_tx_slots_mask;      // Bitmask: slots where this node transmits its own data
  uint8_t      my_relay_slots_mask;   // Bitmask: slots where this node forwards queued data
  bool         schedule_received;
  // Duty cycle / sleep tracking
  uint32_t     active_time_ms;
  uint32_t     sleep_time_ms;
  uint32_t     last_epoch_switch_ms;
  tdma_epoch_t last_epoch;
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
struct fwd_queue_t {
  wsn_packet_t queue[FWD_QUEUE_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
};

// ==================== AGGREGATION STATE (N3 only) ====================
#define AGG_BUFFER_SIZE  8
struct agg_state_t {
  // N1 vibration buffer
  float    n1_ax[AGG_BUFFER_SIZE];
  float    n1_ay[AGG_BUFFER_SIZE];
  float    n1_az[AGG_BUFFER_SIZE];
  uint16_t n1_seq[AGG_BUFFER_SIZE];
  uint8_t  n1_count;
  // N2 flame buffer
  float    n2_analog[AGG_BUFFER_SIZE];
  uint8_t  n2_digital[AGG_BUFFER_SIZE];
  uint16_t n2_seq[AGG_BUFFER_SIZE];
  uint8_t  n2_count;
  // Byte tracking
  uint16_t bytes_received;
};

// ==================== GLOBAL MESH STATE ====================
// Declared here, instantiated in each .ino file
struct mesh_state_t {
  // Identity
  uint8_t      my_id;
  node_role_t  my_role;

  // Neighbor / routing table (index 0 unused, 1..4 for nodes)
  route_neighbor_t neighbors[NUM_NODES + 1];
  uint8_t          next_hop_to_sink;      // Best next hop toward N4
  uint8_t          peer_macs[NUM_NODES + 1][6]; // Dynamically learned peer MACs

  // MAC
  mac_stats_t      mac_stats;
  volatile uint32_t last_rx_time_ms;       // For software CCA
  uint8_t          current_cw;             // Current contention window

  // Volatile ACK state (shared between ISR callback and main loop)
  volatile bool     ack_pending;
  volatile uint16_t ack_expected_seq;
  volatile uint8_t  ack_expected_origin;
  volatile bool     ack_received;

  // TDMA
  tdma_state_t     tdma;

  // Non-blocking forwarding queue (Defect 1 fix)
  fwd_queue_t      fwd_queue;

  // BLE (index 0 unused, 1..4)
  ble_neighbor_t   ble[NUM_NODES + 1];

  // Aggregation (N3 only)
  agg_state_t      agg;

  // Software link kills (for demo failure injection)
  bool             killed_links[NUM_NODES + 1];  // killed_links[x] = true → drop packets to/from node x

  // Separate sequence counters (Defect 6 fix: avoids artificial PDR drops)
  uint16_t         my_seq_counter;        // General counter
  uint16_t         data_seq_counter;      // Dedicated sensor/agg data counter
  uint16_t         ctrl_seq_counter;      // Dedicated control/BLE report counter

  // Performance counters
  uint32_t         pkts_sent;
  uint32_t         pkts_forwarded;
  uint32_t         pkts_received_at_sink;  // Only relevant for N4

  // Timers
  uint32_t         last_hello_ms;
  uint32_t         last_ble_scan_ms;

  // Collision test flag
  volatile bool    collision_test_trigger;
};

// ==================== UTILITY FUNCTIONS ====================

inline const char* nodeIdStr(uint8_t id) {
  static char buf[4];
  if (id == BROADCAST_ID) return "BCAST";
  snprintf(buf, sizeof(buf), "N%d", id);
  return buf;
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

inline void logPacket(const char* dir, const wsn_packet_t& p) {
  Serial.printf("[%s] type=%-9s src=N%d dst=%-5s origin=N%d final=N%d seq=%d hops=%d ttl=%d\n",
    dir, msgTypeStr(p.msg_type),
    p.src_id, nodeIdStr(p.dst_id),
    p.origin_id, p.final_dst_id,
    p.seq_num, p.hop_count, p.ttl);
}

extern mesh_state_t mesh;

// Initialize a packet with standard header fields
inline void initPacket(wsn_packet_t& pkt, uint8_t type, uint8_t dst,
                       uint8_t origin, uint8_t final_dst, uint16_t seq) {
  memset(&pkt, 0, sizeof(pkt));
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

#endif // WSN_COMMON_H
