// ==================== WSN MESH PROTOCOL STACK ====================
// Full mesh stack: MAC (CCA + random backoff + ACK + retransmission),
// Routing (neighbour table + distance-vector with poison reverse +
// forwarding + failure detection/recovery), TDMA (sync + slot engine +
// schedule distribution + sleep accounting).
//
// This file is header-only. Include it after wsn_common.h.
// Every node compiles the same code; role-specific behaviour is
// controlled by MY_NODE_ID and NODE_ROLES[MY_NODE_ID].
//
// Concurrency: ESP-NOW callbacks run in the Wi-Fi task, loop() runs in the
// Arduino task. Shared queues/caches are protected by mesh_mux (short
// critical sections, never around Serial output).
// =================================================================
#ifndef WSN_MESH_H
#define WSN_MESH_H

#include "wsn_common.h"
#if WSN_ENABLE_LIGHT_SLEEP
#include "esp_sleep.h"
#endif

// ==================== GLOBAL MESH STATE ====================
// Single instance per node (one compilation unit per Arduino sketch)
mesh_state_t mesh;
portMUX_TYPE mesh_mux = portMUX_INITIALIZER_UNLOCKED;

// Serial log level: 0 = summaries only, 1 = events + 5 s summaries (default),
// 2 = full per-packet trace. Changed at runtime with the 'log <0|1|2>' command.
uint8_t  wsn_log_level = 1;
uint32_t wsn_log_force_until = 0;   // temporary full trace (e.g. during the collision test)

// ==================== FORWARD DECLARATIONS ====================
void mesh_handle_data(const wsn_packet_t& pkt, int rssi);
void mesh_handle_ack(const wsn_packet_t& pkt);
void mesh_handle_hello(const wsn_packet_t& pkt, int rssi);
void mesh_handle_beacon(const wsn_packet_t& pkt, int rssi);
void mesh_handle_schedule(const wsn_packet_t& pkt);
void mesh_handle_route_update(const wsn_packet_t& pkt);
void mesh_handle_ctrl(const wsn_packet_t& pkt);
void mesh_handle_ble_report(const wsn_packet_t& pkt, int rssi);
void mesh_handle_agg(const wsn_packet_t& pkt, int rssi);

void routing_recompute_next_hop();
void routing_mark_link_dead(uint8_t nb, const char* reason);
void routing_send_hello();
void routing_check_timeouts();
mac_result_t mac_send(wsn_packet_t& pkt, uint8_t& attempts, uint32_t deadline_ms);
bool mac_send_broadcast(wsn_packet_t& pkt, bool log);
void mac_send_ack(const wsn_packet_t& received_pkt);
bool espnow_raw_send(uint8_t dst_id, const wsn_packet_t& pkt);
void tdma_apply_schedule(const schedule_payload_t& sp, bool local);

// Application-level callbacks (implemented in each node's .ino)
extern void app_on_data_for_me(const wsn_packet_t& pkt, int rssi);
extern void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2);
// Called when this node is an intermediate hop for a DATA packet.
// Return TRUE to consume the packet (e.g., N3 buffers it for aggregation).
// Return FALSE to let the mesh layer forward it normally.
extern bool app_on_data_transit(const wsn_packet_t& pkt, int rssi);

// =====================================================================
//                     TIME / EPOCH HELPERS
// =====================================================================
static const uint16_t EPOCH_END_MS[7] = {
  BEACON_START_MS + BEACON_DURATION_MS, CAP_START_MS + CAP_DURATION_MS,
  SLOT1_START_MS + SLOT1_DURATION_MS,   SLOT2_START_MS + SLOT2_DURATION_MS,
  SLOT3_START_MS + SLOT3_DURATION_MS,   SLOT4_START_MS + SLOT4_DURATION_MS,
  SUPERFRAME_MS
};

uint32_t tdma_synced_time() {
  return millis() + mesh.tdma.clock_offset;
}

tdma_epoch_t tdma_epoch_at(uint32_t t_in_frame) {
  for (uint8_t e = EPOCH_BEACON; e <= EPOCH_SLEEP; e++) {
    if (t_in_frame < EPOCH_END_MS[e]) return (tdma_epoch_t)e;
  }
  return EPOCH_SLEEP;
}

// --- Current epoch within superframe ---
tdma_epoch_t tdma_current_epoch() {
  if (!mesh.tdma.synced) return EPOCH_CAP;  // Unsynced: contention-only operation
  return tdma_epoch_at(tdma_synced_time() % SUPERFRAME_MS);
}

// --- Frame number derived from the synchronised clock (identical on all nodes) ---
uint32_t tdma_current_frame() {
  return mesh.tdma.synced ? (tdma_synced_time() / SUPERFRAME_MS) : 0;
}

// --- Time remaining in current epoch (ms) ---
uint32_t tdma_epoch_remaining_ms() {
  if (!mesh.tdma.synced) return 1000;
  uint32_t t = tdma_synced_time() % SUPERFRAME_MS;
  uint32_t end = EPOCH_END_MS[tdma_epoch_at(t)];
  return (end > t) ? (end - t) : 0;
}

// --- Local millis() deadline for transmissions in the current epoch ---
uint32_t tdma_epoch_deadline() {
  uint32_t r = tdma_epoch_remaining_ms();
  return millis() + ((r > GUARD_MS) ? (r - GUARD_MS) : 0);
}

// --- Is this my TX slot? (Transmitting own data) ---
bool tdma_is_my_tx_slot() {
  if (!mesh.tdma.synced) return true;  // Unsynced: free running
  return (mesh.tdma.my_tx_slots_mask & (1 << tdma_current_epoch())) != 0;
}

// --- Is this my RELAY slot? (Forwarding queued transit packets) ---
bool tdma_is_my_relay_slot() {
  if (!mesh.tdma.synced) return true;
  return (mesh.tdma.my_relay_slots_mask & (1 << tdma_current_epoch())) != 0;
}

bool tdma_is_sleep_epoch() {
  return tdma_current_epoch() == EPOCH_SLEEP;
}

// Window for control traffic (HELLO, schedule, route updates, BLE reports, retries)
bool mesh_ctrl_window_open() {
  return !mesh.tdma.synced || tdma_current_epoch() == EPOCH_CAP;
}

// Window for NEW own data: own TX slot, or CAP if this node has no slot
// (schedule not received yet, or slot released by the coordinator).
bool mesh_data_window_open() {
  if (!mesh.tdma.synced) return true;
  if (tdma_is_my_tx_slot()) return true;
  bool no_slot = !mesh.tdma.schedule_received || mesh.tdma.my_tx_slots_mask == 0;
  return no_slot && tdma_current_epoch() == EPOCH_CAP;
}

// Window for retransmitting a pending own packet: own slot or CAP
bool mesh_retry_window_open() {
  return mesh_data_window_open() || mesh_ctrl_window_open();
}

// One NEW own packet per transmission opportunity. Epoch, frame and the
// remaining slot time come from ONE clock reading, so a slow loop iteration
// at a slot boundary can no longer start a transmission in the next node's
// slot (seen on hardware as "S3: TX opportunity" on N2).
bool mesh_take_data_opportunity(uint32_t& last_key) {
  if (!mesh.tdma.synced) {
    uint32_t key = 0x80000000UL | (millis() / 1000);
    if (key == last_key) return false;
    last_key = key;
    return true;
  }
  uint32_t t = tdma_synced_time();
  uint32_t in_frame = t % SUPERFRAME_MS;
  tdma_epoch_t e = tdma_epoch_at(in_frame);
  bool mine = (mesh.tdma.my_tx_slots_mask & (1 << e)) != 0;
  bool no_slot = !mesh.tdma.schedule_received || mesh.tdma.my_tx_slots_mask == 0;
  if (!(mine || (no_slot && e == EPOCH_CAP))) return false;
  if (EPOCH_END_MS[e] - in_frame < ACK_TIMEOUT_MS + 10) return false;   // too late in this slot
  uint32_t key = (t / SUPERFRAME_MS) * 8 + e;
  if (key == last_key) return false;
  last_key = key;
  return true;
}

// Identifies one transmission opportunity (one slot occurrence) so that a node
// generates at most one new packet per opportunity. Unsynced: once per second.
uint32_t mesh_tx_opportunity_key() {
  if (!mesh.tdma.synced) return 0x80000000UL | (millis() / 1000);
  return tdma_current_frame() * 8 + tdma_current_epoch();
}

// =====================================================================
//                     DUPLICATE SUPPRESSION
// =====================================================================
// Returns true if (type, origin, seq) was already seen recently; otherwise
// records it. Used so a retransmission after a lost ACK is re-ACKed but not
// processed/forwarded/aggregated twice.
bool mesh_seen_before(uint8_t type, uint8_t origin, uint16_t seq) {
  uint32_t now = millis();
  bool dup = false;
  portENTER_CRITICAL(&mesh_mux);
  for (int i = 0; i < DUP_CACHE_SIZE; i++) {
    dup_entry_t& e = mesh.dup_cache[i];
    if (e.t_ms != 0 && e.type == type && e.origin == origin && e.seq == seq &&
        (int32_t)(now - e.t_ms) < (int32_t)DUP_CACHE_TTL_MS) {
      dup = true;
      break;
    }
  }
  if (!dup) {
    dup_entry_t& e = mesh.dup_cache[mesh.dup_next];
    e.type = type; e.origin = origin; e.seq = seq; e.t_ms = now ? now : 1;
    mesh.dup_next = (mesh.dup_next + 1) % DUP_CACHE_SIZE;
  }
  portEXIT_CRITICAL(&mesh_mux);
  return dup;
}

// =====================================================================
//                     ESP-NOW RAW SEND HELPER
// =====================================================================
// Low-level send — no CCA, no ACK, no retry. Sends header + payload_len bytes.
bool espnow_raw_send(uint8_t dst_id, const wsn_packet_t& pkt) {
  const uint8_t* mac;
  if (dst_id == BROADCAST_ID) {
    mac = BROADCAST_MAC;
  } else if (dst_id < 1 || dst_id > NUM_NODES) {
    Serial.printf("[ESP-NOW] Invalid destination id %d\n", dst_id);
    return false;
  } else if (mesh.peer_macs[dst_id][0] != 0 || mesh.peer_macs[dst_id][1] != 0) {
    mac = mesh.peer_macs[dst_id];
  } else {
    mac = NODE_MAC[dst_id];
  }

  // Ensure peer exists before unicast send
  if (dst_id != BROADCAST_ID && !esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }

  esp_err_t result = esp_now_send(mac, (const uint8_t*)&pkt, wsn_wire_len(pkt));
  if (result == ESP_ERR_ESPNOW_NO_MEM) {   // driver TX queue momentarily full → one retry
    yield();
    delay(2);
    result = esp_now_send(mac, (const uint8_t*)&pkt, wsn_wire_len(pkt));
  }
  if (result != ESP_OK) {
    Serial.printf("[ESP-NOW] Raw send FAILED to %s (err=%d)\n", nodeIdStr(dst_id), result);
    return false;
  }
  return true;
}

// =====================================================================
//                          MAC LAYER
// =====================================================================
// Application-level CSMA/CA on top of ESP-NOW:
//   1. channel sensing (software CCA: WSN frames sniffed in promiscuous mode)
//   2. random backoff in [0, CW-1] slots (CW doubles per attempt: BEB)
//   3. re-sense: if a WSN frame was heard during the backoff → defer, retry
//   4. transmit, wait for application ACK (matched on seq+origin+type+sender)
//   5. ACK timeout → retransmit (max MAC_MAX_ATTEMPTS attempts in total)
// The ESP32 Wi-Fi hardware additionally runs 802.11 DCF below ESP-NOW;
// the steps above are the custom, observable application-level MAC.

// --- Software Clear Channel Assessment (CCA) ---
bool mac_channel_busy(uint32_t since_ms) {
  uint32_t last = mesh.last_rx_time_ms;
  if (since_ms != 0 && (int32_t)(last - since_ms) >= 0) return true;  // heard during backoff
  return (millis() - last) < CCA_BUSY_THRESHOLD_MS;
}

bool mac_cca(uint32_t since_ms = 0) {
  if (mac_channel_busy(since_ms)) {
    mesh.mac_stats.cca_busy_count++;
    LOGV("[MAC] CCA: BUSY (WSN frame sniffed %lu ms ago)\n",
                  (unsigned long)wsn_age_ms(mesh.last_rx_time_ms));
    return false;
  }
  return true;
}

// --- Random backoff with the given contention window ---
uint32_t mac_backoff(uint8_t cw) {
  uint8_t slots = random(0, cw);
  uint32_t wait_ms = slots * BACKOFF_SLOT_MS;
  mesh.mac_stats.total_backoffs++;
  LOGV("[MAC] Backoff: CW=%d slots=%d wait=%lu ms\n", cw, slots, (unsigned long)wait_ms);
  if (wait_ms) delay(wait_ms);
  return wait_ms;
}

// --- Broadcast with channel sensing (no ACK by definition) ---
bool mac_send_broadcast(wsn_packet_t& pkt, bool log) {
  pkt.src_id = MY_NODE_ID;
  pkt.dst_id = BROADCAST_ID;
  if (mac_channel_busy(0)) {
    mesh.mac_stats.cca_busy_count++;
    mac_backoff(CW_MIN);
  }
  if (log) logPacket("TX", pkt);
  mesh.pkts_sent++;
  return espnow_raw_send(BROADCAST_ID, pkt);
}

// --- Unicast with CSMA/CA + application ACK + retransmission ---
// attempts: attempts already used for this packet (kept by the caller so a
//           packet can continue in the next slot/CAP after MAC_DEFERRED)
// deadline_ms: local millis() by which the exchange must be complete
mac_result_t mac_send(wsn_packet_t& pkt, uint8_t& attempts, uint32_t deadline_ms) {
  if (pkt.dst_id == BROADCAST_ID) {
    return mac_send_broadcast(pkt, true) ? MAC_OK : MAC_DROPPED;
  }
  pkt.src_id = MY_NODE_ID;
  uint8_t busy_rounds = 0;

  while (attempts < MAC_MAX_ATTEMPTS) {
    if (pkt.dst_id < 1 || pkt.dst_id > NUM_NODES) return MAC_NO_ROUTE;

    uint8_t exp = attempts + busy_rounds;
    if (exp > 4) exp = 4;
    uint8_t cw = (uint8_t)min((int)(CW_MIN << exp), (int)CW_MAX);
    mesh.current_cw = cw;

    // Slot boundary: the whole exchange (backoff + frame + ACK wait) must fit
    int32_t left = (int32_t)(deadline_ms - millis());
    int32_t need = (cw - 1) * BACKOFF_SLOT_MS + ACK_TIMEOUT_MS + 2;
    if (left < need) {
      mesh.mac_stats.total_deferrals++;
      LOGV("[MAC] DEFER seq=%d: %ld ms left in %s < %ld ms needed → continue in next slot/CAP\n",
                    pkt.seq_num, (long)(left > 0 ? left : 0), epochStr(tdma_current_epoch()), (long)need);
      return MAC_DEFERRED;
    }

    // 1. Channel sensing
    bool idle = mac_cca();
    LOGV("[MAC] CSMA %s seq=%d → N%d attempt %d/%d: channel %s\n",
                  msgTypeStr(pkt.msg_type), pkt.seq_num, pkt.dst_id,
                  attempts + 1, MAC_MAX_ATTEMPTS, idle ? "IDLE" : "BUSY");

    // 2. Random backoff (always, so simultaneous senders de-synchronise)
    uint32_t bo_start = millis();
    mac_backoff(idle ? cw : (uint8_t)min((int)cw * 2, (int)CW_MAX));

    // 3. Re-sense after backoff: someone else won the contention → defer
    if (!mac_cca(bo_start)) {
      busy_rounds++;
      LOGV("[MAC] Contention: channel taken during backoff → defer seq=%d (CW doubles)\n",
                    pkt.seq_num);
      continue;
    }

    // 4. Arm ACK expectation
    mesh.ack_expected_seq    = pkt.seq_num;
    mesh.ack_expected_origin = pkt.origin_id;
    mesh.ack_expected_type   = pkt.msg_type;
    mesh.ack_expected_from   = pkt.dst_id;
    mesh.ack_received        = false;
    mesh.ack_pending         = true;

    // 5. Transmit
    attempts++;
    mesh.mac_stats.total_tx_attempts++;
    if (attempts > 1) {
      mesh.mac_stats.total_retries++;
      LOGE("[MAC] RETRANSMIT seq=%d to N%d (attempt %d/%d)\n",
                    pkt.seq_num, pkt.dst_id, attempts, MAC_MAX_ATTEMPTS);
    }
    logPacket("TX", pkt);
    bool sent = espnow_raw_send(pkt.dst_id, pkt);
    mesh.pkts_sent++;

    uint32_t ack_start = millis();
    if (sent) {
      while (!mesh.ack_received && (millis() - ack_start) < ACK_TIMEOUT_MS) {
        delayMicroseconds(200);
      }
    }
    mesh.ack_pending = false;

    uint8_t nb = pkt.dst_id;
    route_neighbor_t& n = mesh.neighbors[nb];
    n.ack_sent++;

    if (mesh.ack_received) {
      uint32_t rtt = millis() - ack_start;
      mesh.mac_stats.total_acks_ok++;
      n.ack_received++;
      n.consec_ack_fails = 0;
      n.ack_success_rate = 0.2f * 1.0f + 0.8f * n.ack_success_rate;
      LOGV("[MAC] ACK OK seq=%d from N%d (RTT=%lu ms, attempt %d)\n",
                    pkt.seq_num, nb, (unsigned long)rtt, attempts);
      return MAC_OK;
    }

    // 6. ACK timeout
    mesh.mac_stats.total_acks_timeout++;
    n.consec_ack_fails++;
    n.ack_success_rate = 0.8f * n.ack_success_rate;
    LOGE("[MAC] ACK TIMEOUT seq=%d from N%d after %d ms (attempt %d/%d, consecutive=%d)\n",
                  pkt.seq_num, nb, ACK_TIMEOUT_MS, attempts, MAC_MAX_ATTEMPTS, n.consec_ack_fails);

    // Failure detection: repeated ACK timeouts → link dead → reroute this packet
    if (n.consec_ack_fails >= LINK_FAIL_ACK_TIMEOUTS && n.link_alive) {
      routing_mark_link_dead(nb, "consecutive ACK timeouts");
      uint8_t alt = mesh.next_hop_to_sink;
      if (pkt.final_dst_id == SINK_ID && alt != 0 && alt != nb) {
        pkt.dst_id = alt;
        mesh.mac_stats.total_reroutes++;
        if (attempts > MAC_MAX_ATTEMPTS - 2) attempts = MAC_MAX_ATTEMPTS - 2;
        LOGE("[ROUTE] Re-routing seq=%d (origin N%d) via ALTERNATE next hop N%d\n",
                      pkt.seq_num, pkt.origin_id, alt);
      }
    }
  }

  // All attempts used
  mesh.mac_stats.total_drops++;
  LOGE("[MAC] DROPPED seq=%d to N%d after %d attempts\n",
                pkt.seq_num, pkt.dst_id, MAC_MAX_ATTEMPTS);
  return MAC_DROPPED;
}

// --- Send an ACK for a received unicast packet ---
void mac_send_ack(const wsn_packet_t& received_pkt) {
  wsn_packet_t ack;
  initPacket(ack, MSG_ACK, received_pkt.src_id, MY_NODE_ID, received_pkt.src_id, 0);
  ack_payload_t ap;
  ap.acked_seq_num   = received_pkt.seq_num;
  ap.acked_origin_id = received_pkt.origin_id;
  ap.acked_type      = received_pkt.msg_type;
  setPayload(ack, ap);
  // ACKs are sent immediately and raw (they are part of the sender's exchange)
  espnow_raw_send(received_pkt.src_id, ack);
}

// =====================================================================
//                        ROUTING LAYER
// =====================================================================

// --- Initialize neighbour table ---
void routing_init() {
  for (int i = 0; i <= NUM_NODES; i++) {
    mesh.neighbors[i].node_id = i;
    mesh.neighbors[i].wifi_rssi = -100;
    mesh.neighbors[i].ack_success_rate = 0.5f;
    mesh.neighbors[i].hops_to_sink = HOPS_INFINITY;
    mesh.neighbors[i].next_hop = 0;
    mesh.neighbors[i].last_hello_ms = 0;
    mesh.neighbors[i].link_alive = false;
    mesh.neighbors[i].ack_sent = 0;
    mesh.neighbors[i].ack_received = 0;
    mesh.neighbors[i].consec_ack_fails = 0;
  }
  if (MY_NODE_ID == SINK_ID) {
    mesh.neighbors[SINK_ID].hops_to_sink = 0;
    mesh.neighbors[SINK_ID].link_alive = true;
    mesh.next_hop_to_sink = SINK_ID;  // We ARE the sink
  }
}

uint8_t routing_my_hops() {
  if (MY_NODE_ID == SINK_ID) return 0;
  uint8_t nh = mesh.next_hop_to_sink;
  if (nh == 0) return HOPS_INFINITY;
  uint8_t h = mesh.neighbors[nh].hops_to_sink;
  return (h >= HOPS_INFINITY - 1) ? HOPS_INFINITY : h + 1;
}

// Cluster head N3 can be used by N1/N2 (alive, has a route, not routing via us)
bool routing_cluster_head_usable() {
  route_neighbor_t& c = mesh.neighbors[CLUSTER_HEAD_ID];
  return c.link_alive && !mesh.killed_links[CLUSTER_HEAD_ID] &&
         c.hops_to_sink != HOPS_INFINITY && c.next_hop != MY_NODE_ID;
}

// --- Compute route cost to reach sink through neighbour i ---
// Metrics: hop count, ETX (from ACK success rate), Wi-Fi RSSI, link availability.
float routing_cost(uint8_t neighbor_id) {
  route_neighbor_t& n = mesh.neighbors[neighbor_id];
  if (!n.link_alive || mesh.killed_links[neighbor_id] || n.hops_to_sink == HOPS_INFINITY) {
    return ROUTE_INFINITY;
  }
  // Poison reverse / split horizon: a neighbour whose route goes through us
  // must not be used, otherwise a two-node loop forms (N3↔N1 after N3–N4 fails).
  if (neighbor_id != SINK_ID && n.next_hop == MY_NODE_ID) return ROUTE_INFINITY;

  float etx = (n.ack_success_rate > 0.05f) ? (1.0f / n.ack_success_rate) : 20.0f;
  float rssi_norm = (float)(-n.wifi_rssi) / 100.0f;
  float cost = ROUTE_COST_W_HOP * (n.hops_to_sink + 1) +
               ROUTE_COST_W_ETX * etx +
               ROUTE_COST_W_RSSI * rssi_norm;

  // Cluster preference: N1 & N2 route through cluster head N3 for aggregation
  // while N3 is usable; the penalty disappears when N3 fails → direct failover.
  if ((MY_NODE_ID == 1 || MY_NODE_ID == 2) && neighbor_id == SINK_ID && routing_cluster_head_usable()) {
    cost += ROUTE_CLUSTER_PENALTY;
  }
  // Prefer the designated relay (N2) over the leaf sensor (N1) as an intermediate hop
  if (neighbor_id != SINK_ID && NODE_ROLES[neighbor_id] == ROLE_SENSOR) {
    cost += ROUTE_LEAF_RELAY_BIAS;
  }
  return cost;
}

// --- Best next hop toward the sink, ignoring one neighbour (0 = none) ---
uint8_t routing_best_hop_excluding(uint8_t excluded) {
  if (MY_NODE_ID == SINK_ID) return SINK_ID;
  float best = ROUTE_INFINITY;
  uint8_t hop = 0;
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID || i == excluded) continue;
    float c = routing_cost(i);
    if (c < best) { best = c; hop = i; }
  }
  return hop;
}

// --- Recompute best next hop toward the sink ---
void routing_recompute_next_hop() {
  if (MY_NODE_ID == SINK_ID) {
    mesh.next_hop_to_sink = SINK_ID;
    return;
  }

  float best_cost = ROUTE_INFINITY;
  uint8_t best_hop = 0;
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    float c = routing_cost(i);
    if (c < best_cost) {
      best_cost = c;
      best_hop = i;
    }
  }

  uint8_t old = mesh.next_hop_to_sink;
  mesh.next_hop_to_sink = best_hop;
  mesh.next_hop_cost = best_cost;
  if (best_hop != old) {
    if (best_hop != 0) {
      LOGE("[ROUTE] Next hop changed: %s → N%d (cost=%.2f, hops_to_sink=%d)\n",
                    old ? nodeIdStr(old) : "none", best_hop, best_cost, routing_my_hops());
    } else {
      LOGE("[ROUTE] WARNING: No route to sink!\n");
    }
    mesh.hello_triggered = true;  // Announce the new route quickly (triggered update)
  }
}

void routing_queue_route_update(uint8_t a, uint8_t b, bool up) {
  mesh.route_update_payload.node_a = a;
  mesh.route_update_payload.node_b = b;
  mesh.route_update_payload.link_up = up ? 1 : 0;
  mesh.route_update_pending = true;
}

// --- Failure detection outcome: mark link dead, announce, reroute ---
void routing_mark_link_dead(uint8_t nb, const char* reason) {
  if (nb < 1 || nb > NUM_NODES || nb == MY_NODE_ID) return;
  bool was_alive = mesh.neighbors[nb].link_alive;
  mesh.neighbors[nb].link_alive = false;
  if (was_alive) {
    LOGE("[ROUTE] *** Link N%d→N%d DEAD (%s) — failure detected ***\n",
                  MY_NODE_ID, nb, reason);
    routing_queue_route_update(MY_NODE_ID, nb, false);
  }
  routing_recompute_next_hop();
}

// Mark a neighbour alive after hearing it (HELLO or BEACON)
void routing_note_neighbor_alive(uint8_t nid, int rssi) {
  route_neighbor_t& nb = mesh.neighbors[nid];
  bool was_dead = !nb.link_alive;
  nb.link_alive = true;
  nb.consec_ack_fails = 0;
  nb.last_hello_ms = millis();
  nb.wifi_rssi = (int8_t)rssi;
  if (was_dead) {
    LOGE("[ROUTE] Link N%d↔N%d UP (heard %s, RSSI=%d)\n",
                  MY_NODE_ID, nid, nid == SINK_ID ? "sink" : "HELLO", rssi);
    routing_queue_route_update(MY_NODE_ID, nid, true);
  }
}

// --- Build and broadcast a HELLO packet ---
void routing_send_hello() {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_HELLO, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, 0);

  hello_payload_t hp;
  memset(&hp, 0, sizeof(hp));
  hp.node_id = MY_NODE_ID;
  hp.hops_to_sink = routing_my_hops();
  hp.next_hop = mesh.next_hop_to_sink;
  uint8_t nh = mesh.next_hop_to_sink;
  hp.ack_success_rate = (nh != 0 && nh != MY_NODE_ID) ? mesh.neighbors[nh].ack_success_rate : 1.0f;
  hp.wifi_rssi = (nh != 0 && nh != MY_NODE_ID) ? mesh.neighbors[nh].wifi_rssi : 0;
  hp.node_retries = (uint16_t)min((uint32_t)65535, mesh.mac_stats.total_retries);
  hp.cca_busy     = (uint16_t)min((uint32_t)65535, mesh.mac_stats.cca_busy_count);
  hp.backoffs     = (uint16_t)min((uint32_t)65535, mesh.mac_stats.total_backoffs);
  hp.ack_timeouts = (uint16_t)min((uint32_t)65535, mesh.mac_stats.total_acks_timeout);
  hp.drops        = (uint16_t)min((uint32_t)65535, mesh.mac_stats.total_drops);
  hp.synced = mesh.tdma.synced ? 1 : 0;
  hp.synced_time_ms = tdma_synced_time();
  uint32_t total = mesh.tdma.active_time_ms + mesh.tdma.sleep_time_ms;
  hp.duty_pct = total ? (uint8_t)(mesh.tdma.active_time_ms * 100ULL / total) : 100;

  setPayload(pkt, hp);
  mac_send_broadcast(pkt, false);  // Broadcast: no ACK
  mesh.last_hello_ms = millis();
  mesh.hello_triggered = false;
}

// --- Process incoming HELLO ---
void mesh_handle_hello(const wsn_packet_t& pkt, int rssi) {
  hello_payload_t hp;
  getPayload(pkt, hp);
  uint8_t nid = hp.node_id;
  if (nid < 1 || nid > NUM_NODES || nid == MY_NODE_ID || nid != pkt.src_id) return;
  if (mesh.killed_links[nid]) return;

  routing_note_neighbor_alive(nid, rssi);
  route_neighbor_t& nb = mesh.neighbors[nid];
  nb.hops_to_sink = (nid == SINK_ID) ? 0 : hp.hops_to_sink;
  nb.next_hop = hp.next_hop;
  nb.reported_retries = hp.node_retries;
  nb.reported_cca_busy = hp.cca_busy;
  nb.reported_backoffs = hp.backoffs;
  nb.reported_ack_timeouts = hp.ack_timeouts;
  nb.reported_drops = hp.drops;
  nb.reported_duty_pct = hp.duty_pct;

  // Multi-hop time sync: if the sink's beacon is stale (e.g. our link to N4
  // failed), adopt the clock of a synced neighbour that still has a route.
  if (MY_NODE_ID != SINK_ID && hp.synced && hp.hops_to_sink != HOPS_INFINITY &&
      (!mesh.tdma.synced || wsn_age_ms(mesh.tdma.last_beacon_rx_ms) > BEACON_STALE_MS)) {
    bool was_synced = mesh.tdma.synced;
    mesh.tdma.clock_offset = (int32_t)hp.synced_time_ms - (int32_t)millis();
    mesh.tdma.synced = true;
    mesh.tdma.synced_from_beacon = false;
    mesh.tdma.last_sync_ms = millis();
    if (!was_synced) {
      LOGE("[TDMA] Synchronised via N%d HELLO (multi-hop time sync), offset=%ld ms\n",
                    nid, (long)mesh.tdma.clock_offset);
    }
  }

  routing_recompute_next_hop();
}

// --- Queue a transit packet for forwarding (called from RX callback) ---
bool mesh_enqueue_forward(const wsn_packet_t& in) {
  wsn_packet_t p = in;
  if (p.ttl == 0) {
    LOGE("[ROUTE] Dropping seq=%d (origin N%d) — TTL expired\n", p.seq_num, p.origin_id);
    return false;
  }
  // Loop guard: drop if this node already appears in the recorded path
  uint8_t last = (p.hop_count < 4) ? p.hop_count : 3;
  for (uint8_t k = 0; k <= last; k++) {
    if (p.route_path[k] == MY_NODE_ID) {
      LOGE("[ROUTE] Dropping seq=%d (origin N%d) — routing loop detected\n", p.seq_num, p.origin_id);
      return false;
    }
  }
  p.ttl--;
  p.hop_count++;
  if (p.hop_count < 4) p.route_path[p.hop_count] = MY_NODE_ID;

  bool ok = false;
  uint8_t depth = 0;
  portENTER_CRITICAL(&mesh_mux);
  if (mesh.fwd_queue.count < FWD_QUEUE_SIZE) {
    mesh.fwd_queue.queue[mesh.fwd_queue.head].pkt = p;
    mesh.fwd_queue.queue[mesh.fwd_queue.head].attempts = 0;
    mesh.fwd_queue.head = (mesh.fwd_queue.head + 1) % FWD_QUEUE_SIZE;
    mesh.fwd_queue.count++;
    depth = mesh.fwd_queue.count;
    ok = true;
  }
  portEXIT_CRITICAL(&mesh_mux);
  if (ok) {
    LOGV("[ROUTE] Queued %s seq=%d (origin N%d) for forwarding (queue=%d)\n",
                  msgTypeStr(p.msg_type), p.seq_num, p.origin_id, depth);
  } else {
    LOGE("[ROUTE] Fwd queue full — dropping packet\n");
  }
  return ok;
}

// --- Forward queued packets (main loop; relay slot or CAP) ---
void mesh_service_forward_queue() {
  bool can_forward = (!mesh.tdma.synced) || tdma_is_my_relay_slot() || (tdma_current_epoch() == EPOCH_CAP);
  if (!can_forward) return;
  static uint32_t defer_key = 0xFFFFFFFF;
  if (defer_key == mesh_tx_opportunity_key()) return;   // already deferred in this window

  while (true) {
    fwd_entry_t e;
    bool have = false;
    portENTER_CRITICAL(&mesh_mux);
    if (mesh.fwd_queue.count > 0) {
      e = mesh.fwd_queue.queue[mesh.fwd_queue.tail];
      have = true;
    }
    portEXIT_CRITICAL(&mesh_mux);
    if (!have) break;

    uint8_t next = mesh.next_hop_to_sink;
    mac_result_t r;
    if (next == 0) {
      LOGE("[ROUTE] Cannot forward seq=%d (origin N%d) — no route to sink\n", e.pkt.seq_num, e.pkt.origin_id);
      r = MAC_NO_ROUTE;
    } else if (next == e.pkt.src_id && routing_best_hop_excluding(e.pkt.src_id) == 0) {
      LOGE("[ROUTE] Not returning seq=%d to previous hop N%d and no other route (loop avoided) — dropped\n", e.pkt.seq_num, next);
      r = MAC_NO_ROUTE;
    } else {
      if (next == e.pkt.src_id) {
        // Our route just flipped back toward the node that handed us this packet
        // (e.g. after 'restore'): use the best OTHER neighbour instead of dropping.
        next = routing_best_hop_excluding(e.pkt.src_id);
        LOGE("[ROUTE] seq=%d came from N%d — sending it via N%d instead (no ping-pong)\n",
             e.pkt.seq_num, e.pkt.src_id, next);
      }
      e.pkt.dst_id = next;
      LOGV("[ROUTE] Forwarding %s seq=%d (origin=N%d) via N%d\n",
                    msgTypeStr(e.pkt.msg_type), e.pkt.seq_num, e.pkt.origin_id, next);
      r = mac_send(e.pkt, e.attempts, tdma_epoch_deadline());
    }

    portENTER_CRITICAL(&mesh_mux);
    if (r == MAC_DEFERRED) {
      mesh.fwd_queue.queue[mesh.fwd_queue.tail] = e;  // keep attempts for next window
    } else {
      mesh.fwd_queue.tail = (mesh.fwd_queue.tail + 1) % FWD_QUEUE_SIZE;
      mesh.fwd_queue.count--;
    }
    portEXIT_CRITICAL(&mesh_mux);

    if (r == MAC_OK) mesh.pkts_forwarded++;
    if (r == MAC_DEFERRED) { defer_key = mesh_tx_opportunity_key(); break; }
  }
}

// --- Check neighbour timeouts ---
void routing_check_timeouts() {
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    route_neighbor_t& nb = mesh.neighbors[i];
    if (nb.link_alive && nb.last_hello_ms > 0 &&
        wsn_age_ms(nb.last_hello_ms) > HELLO_TIMEOUT_MS) {
      char reason[40];
      snprintf(reason, sizeof(reason), "HELLO timeout %lu ms", (unsigned long)wsn_age_ms(nb.last_hello_ms));
      routing_mark_link_dead(i, reason);
    }
  }
}

// --- Handle route update broadcast ---
void mesh_handle_route_update(const wsn_packet_t& pkt) {
  route_update_payload_t ru;
  getPayload(pkt, ru);
  LOGE("[ROUTE] Route update from N%d: Link N%d↔N%d %s\n",
                pkt.src_id, ru.node_a, ru.node_b, ru.link_up ? "UP" : "DOWN");
  routing_recompute_next_hop();
}

// =====================================================================
//                         TDMA LAYER
// =====================================================================

void tdma_init() {
  memset(&mesh.tdma, 0, sizeof(mesh.tdma));
  mesh.tdma.my_tx_slot = 0xFF;
  mesh.tdma.my_relay_slot = 0xFF;
  mesh.tdma.last_epoch_switch_ms = millis();
  mesh.tdma.last_epoch = EPOCH_SLEEP;
  mesh.tdma.last_epoch_active = true;
}

// --- Process incoming beacon from N4 ---
void mesh_handle_beacon(const wsn_packet_t& pkt, int rssi) {
  if (pkt.origin_id != SINK_ID || pkt.src_id != SINK_ID) return;
  beacon_payload_t bp;
  getPayload(pkt, bp);

  bool first = !mesh.tdma.synced || !mesh.tdma.synced_from_beacon;
  int32_t new_offset = (int32_t)bp.timestamp_ms - (int32_t)millis();
  int32_t correction = new_offset - mesh.tdma.clock_offset;
  mesh.tdma.clock_offset = new_offset;
  mesh.tdma.frame_number = bp.frame_number;
  mesh.tdma.synced = true;
  mesh.tdma.synced_from_beacon = true;
  mesh.tdma.last_beacon_rx_ms = millis();
  mesh.tdma.last_sync_ms = millis();

  // Sink heard directly: N4 is a 0-hop neighbour
  routing_note_neighbor_alive(SINK_ID, rssi);
  mesh.neighbors[SINK_ID].hops_to_sink = 0;
  mesh.neighbors[SINK_ID].next_hop = SINK_ID;
  routing_recompute_next_hop();

  if (first) {
    uint8_t ch = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    LOGE("[TDMA] Beacon sync: frame=%lu offset=%ld ms | Channel %d LOCKED\n",
                  (unsigned long)bp.frame_number, (long)mesh.tdma.clock_offset, ch);
  } else if (bp.frame_number % 25 == 0) {
    LOGV("[TDMA] Beacon frame=%lu (clock correction %+ld ms)\n",
                  (unsigned long)bp.frame_number, (long)correction);
  }
}

// --- N4: Broadcast a time-sync beacon (called in the BEACON epoch) ---
void tdma_send_beacon() {
  if (MY_NODE_ID != SINK_ID) return;
  wsn_packet_t pkt;
  initPacket(pkt, MSG_BEACON, BROADCAST_ID, SINK_ID, BROADCAST_ID, 0);
  beacon_payload_t bp;
  bp.frame_number = tdma_current_frame();
  bp.timestamp_ms = millis();  // N4 is the time reference (offset 0)
  setPayload(pkt, bp);
  mac_send_broadcast(pkt, false);
  mesh.tdma.frame_number = bp.frame_number;
}

// --- Human-readable slot table ---
void tdma_schedule_to_string(const schedule_payload_t& sp, char* out, size_t n) {
  out[0] = '\0';
  for (uint8_t e = EPOCH_SLOT1; e <= EPOCH_SLOT4; e++) {
    char part[20] = "";
    bool found = false;
    for (int i = 0; i < sp.slot_count && i < 8; i++) {
      if (sp.slots[i].slot_index != e) continue;
      found = true;
      if (sp.slots[i].node_id == 0 || sp.slots[i].slot_type == SLOT_SLEEP) {
        snprintf(part, sizeof(part), "%s=UNUSED ", epochStr(e));
      } else {
        snprintf(part, sizeof(part), "%s=N%d:%s ", epochStr(e), sp.slots[i].node_id,
                 sp.slots[i].slot_type == SLOT_RELAY ? "RELAY" : "TX");
      }
      break;
    }
    if (!found) snprintf(part, sizeof(part), "%s=UNUSED ", epochStr(e));
    strncat(out, part, n - strlen(out) - 1);
  }
}

// --- Apply a schedule (received, or local from the coordinator) ---
void tdma_apply_schedule(const schedule_payload_t& sp, bool local) {
  uint32_t now = millis();
  if (sp.issuer == CLUSTER_HEAD_ID) mesh.tdma.last_primary_sched_ms = now;

  // Conflict resolution: N3 (primary) wins; N4 failover schedules are only
  // accepted when no N3 schedule has been heard recently.
  if (!local && sp.issuer == SINK_ID) {
    if (MY_NODE_ID == CLUSTER_HEAD_ID) {
      mesh.sched_announce_pending = true;  // re-assert primary schedule
      return;
    }
    if (mesh.tdma.last_primary_sched_ms != 0 &&
        wsn_age_ms(mesh.tdma.last_primary_sched_ms) < SCHED_PRIMARY_TIMEOUT_MS) {
      return;
    }
  }
  mesh.tdma.last_sched_rx_ms = now;

  bool changed = !mesh.tdma.schedule_received || sp.version != mesh.tdma.sched_version ||
                 sp.issuer != mesh.tdma.sched_issuer;
  if (!changed) return;  // periodic refresh of the same schedule

  mesh.tdma.my_tx_slot = 0xFF;
  mesh.tdma.my_relay_slot = 0xFF;
  mesh.tdma.my_tx_slots_mask = 0;
  mesh.tdma.my_relay_slots_mask = 0;
  memset(mesh.tdma.slot_owner, 0, sizeof(mesh.tdma.slot_owner));
  mesh.tdma.schedule_received = true;
  mesh.tdma.sched_version = sp.version;
  mesh.tdma.sched_issuer = sp.issuer;
  strncpy(mesh.tdma.sched_reason, sp.update_reason, sizeof(mesh.tdma.sched_reason) - 1);
  mesh.tdma.sched_reason[sizeof(mesh.tdma.sched_reason) - 1] = '\0';

  for (int i = 0; i < sp.slot_count && i < 8; i++) {
    uint8_t idx = sp.slots[i].slot_index;
    if (idx < EPOCH_SLOT1 || idx > EPOCH_SLOT4) continue;
    if (sp.slots[i].slot_type == SLOT_SLEEP || sp.slots[i].node_id == 0) continue;
    mesh.tdma.slot_owner[idx] = sp.slots[i].node_id;
    if (sp.slots[i].node_id != MY_NODE_ID) continue;
    if (sp.slots[i].slot_type == SLOT_TX) {
      mesh.tdma.my_tx_slots_mask |= (1 << idx);
      mesh.tdma.my_tx_slot = idx;
    } else if (sp.slots[i].slot_type == SLOT_RELAY) {
      mesh.tdma.my_relay_slots_mask |= (1 << idx);
      mesh.tdma.my_relay_slot = idx;
    }
  }

  char table[96];
  tdma_schedule_to_string(sp, table, sizeof(table));
  LOGE("[TDMA] Schedule v%u from N%d applied (%s): %s\n",
                sp.version, sp.issuer, sp.update_reason, table);
  char mine[48] = "";
  for (uint8_t e = EPOCH_SLOT1; e <= EPOCH_SLOT4; e++) {
    char part[16];
    if (mesh.tdma.my_tx_slots_mask & (1 << e)) { snprintf(part, sizeof(part), "%s:TX ", epochStr(e)); strncat(mine, part, sizeof(mine) - strlen(mine) - 1); }
    if (mesh.tdma.my_relay_slots_mask & (1 << e)) { snprintf(part, sizeof(part), "%s:RELAY ", epochStr(e)); strncat(mine, part, sizeof(mine) - strlen(mine) - 1); }
  }
  if (mine[0]) {
    LOGE("[TDMA]   My slots: %s\n", mine);
  } else if (MY_NODE_ID != SINK_ID) {
    LOGE("[TDMA]   No slot assigned to me → own data uses the CAP (contention)\n");
  }
}

void mesh_handle_schedule(const wsn_packet_t& pkt) {
  schedule_payload_t sp;
  getPayload(pkt, sp);
  // N4 relays N3's schedule (origin N4, issuer N3) for nodes that cannot hear N3.
  bool relayed = (pkt.origin_id == SINK_ID && sp.issuer == CLUSTER_HEAD_ID);
  if (sp.issuer != pkt.origin_id && !relayed) return;
  if (MY_NODE_ID == SINK_ID && sp.issuer == CLUSTER_HEAD_ID && !relayed) {
    mesh.last_primary_sched = sp;
    mesh.last_primary_rx_ms = millis();
  }
  if (relayed) {
    if (MY_NODE_ID == CLUSTER_HEAD_ID) return;              // my own schedule echoed back
    if (mesh.tdma.schedule_received && mesh.tdma.sched_issuer == CLUSTER_HEAD_ID &&
        (int16_t)(sp.version - mesh.tdma.sched_version) < 0) return;   // older copy
  }
  tdma_apply_schedule(sp, false);
}

// --- Coordinator: build a schedule from network state ---
// Slot plan: S1=N1 TX, S2=N2 TX, S3=N3 TX (aggregate), S4=N2 RELAY.
// Dynamic changes: absent node → its slot is released (UNUSED);
// N1 vibration alert → S4 given to N1 as an extra TX slot.
void tdma_build_schedule(schedule_payload_t& sp, bool n1, bool n2, bool n3, bool n1_alert,
                         const char* reason, uint16_t version, uint8_t issuer) {
  memset(&sp, 0, sizeof(sp));
  sp.version = version;
  sp.issuer = issuer;
  sp.slot_count = 4;
  sp.slots[0] = { (uint8_t)(n1 ? 1 : 0), EPOCH_SLOT1, (uint8_t)(n1 ? SLOT_TX : SLOT_SLEEP) };
  sp.slots[1] = { (uint8_t)(n2 ? 2 : 0), EPOCH_SLOT2, (uint8_t)(n2 ? SLOT_TX : SLOT_SLEEP) };
  sp.slots[2] = { (uint8_t)(n3 ? 3 : 0), EPOCH_SLOT3, (uint8_t)(n3 ? SLOT_TX : SLOT_SLEEP) };
  if (n1 && n1_alert)  sp.slots[3] = { 1, EPOCH_SLOT4, SLOT_TX };
  else if (n2)         sp.slots[3] = { 2, EPOCH_SLOT4, SLOT_RELAY };
  else                 sp.slots[3] = { 0, EPOCH_SLOT4, SLOT_SLEEP };
  strncpy(sp.update_reason, reason, sizeof(sp.update_reason) - 1);
}

// --- Coordinator: adopt and announce a new schedule ---
void tdma_publish_schedule(const schedule_payload_t& sp) {
  mesh.sched_current = sp;
  mesh.sched_is_coordinator = true;
  mesh.sched_announce_pending = true;
  tdma_apply_schedule(sp, true);
}

void tdma_announce_schedule(bool changed) {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_SCHEDULE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, 0);
  setPayload(pkt, mesh.sched_current);
  mac_send_broadcast(pkt, false);
  mesh.last_sched_announce_ms = millis();
  if (changed) {
    LOGE("[TDMA] Broadcast schedule v%u (%s)\n", mesh.sched_current.version, mesh.sched_current.update_reason);
  }
}

// --- Does this node have a TX/RX role in an epoch? (duty-cycle accounting) ---
bool tdma_epoch_is_active(tdma_epoch_t e) {
  if (!mesh.tdma.synced) return true;
  if (e == EPOCH_BEACON || e == EPOCH_CAP) return true;
  if (e == EPOCH_SLEEP) return false;
  uint8_t bit = (1 << e);
  if ((mesh.tdma.my_tx_slots_mask | mesh.tdma.my_relay_slots_mask) & bit) return true;
  uint8_t owner = mesh.tdma.slot_owner[e];
  if (owner == 0 || owner == MY_NODE_ID) return false;              // unused slot
  if (mesh.neighbors[owner].next_hop == MY_NODE_ID) return true;   // I receive from the slot owner
  return false;
}

// --- Note an own TX slot that carried no packet ---
void mesh_note_unused_slot(const char* why) {
  mesh.tdma.unused_slots++;
  if (mesh.tdma.unused_slots % 10 == 1) {
    LOGV("[TDMA] %s unused (%s) → radio idle [unused own slots=%lu]\n",
                  epochStr(tdma_current_epoch()), why, (unsigned long)mesh.tdma.unused_slots);
  }
}

// --- Epoch bookkeeping: duty cycle, frame summary, optional light sleep ---
void tdma_tick() {
  uint32_t now = millis();
  tdma_epoch_t epoch = tdma_current_epoch();
  if (epoch == mesh.tdma.last_epoch) return;

  uint32_t dt = now - mesh.tdma.last_epoch_switch_ms;
  if (mesh.tdma.last_epoch_active) mesh.tdma.active_time_ms += dt;
  else                             mesh.tdma.sleep_time_ms += dt;
  mesh.tdma.last_epoch = epoch;
  mesh.tdma.last_epoch_switch_ms = now;
  mesh.tdma.last_epoch_active = tdma_epoch_is_active(epoch);

  if (!mesh.tdma.synced) return;
  uint32_t frame = tdma_current_frame();
  mesh.tdma.frame_number = frame;

  if (epoch == EPOCH_BEACON && frame - mesh.tdma.last_logged_frame >= TDMA_LOG_EVERY_FRAMES) {
    mesh.tdma.last_logged_frame = frame;
    char idle[40] = "";
    for (uint8_t e = EPOCH_SLOT1; e <= EPOCH_SLEEP; e++) {
      if (!tdma_epoch_is_active((tdma_epoch_t)e)) {
        strncat(idle, epochStr(e), sizeof(idle) - strlen(idle) - 1);
        strncat(idle, " ", sizeof(idle) - strlen(idle) - 1);
      }
    }
    uint32_t total = mesh.tdma.active_time_ms + mesh.tdma.sleep_time_ms;
    float duty = total ? (mesh.tdma.active_time_ms * 100.0f / total) : 100.0f;
    LOGV("[TDMA] Frame %lu | sched v%u | radio idle/sleep in: %s| duty=%.1f%% (active=%lu ms, sleep=%lu ms)\n",
                  (unsigned long)frame, mesh.tdma.sched_version, idle, duty,
                  (unsigned long)mesh.tdma.active_time_ms, (unsigned long)mesh.tdma.sleep_time_ms);
  }

  if (epoch == EPOCH_SLEEP && frame % TDMA_LOG_EVERY_FRAMES == 0) {
    LOGV("[TDMA] SLEEP epoch: no Wi-Fi TX for %d ms (wake at next BEACON)\n", SLEEP_DURATION_MS);
  }

#if WSN_ENABLE_LIGHT_SLEEP
  // Optional real light sleep for the leaf sensor in the SLEEP epoch.
  // UNVERIFIED on hardware: BLE controller / ESP-NOW behaviour must be checked.
  if (epoch == EPOCH_SLEEP && MY_NODE_ID == 1 && (now - mesh.last_ble_scan_ms) > BLE_SCAN_DURATION_MS + 20) {
    uint32_t rem = tdma_epoch_remaining_ms();
    if (rem > 20) {
      esp_sleep_enable_timer_wakeup((uint64_t)(rem - 10) * 1000ULL);
      esp_err_t err = esp_light_sleep_start();
      if (err != ESP_OK && frame % TDMA_LOG_EVERY_FRAMES == 0) {
        Serial.printf("[TDMA] Light sleep rejected: %s\n", esp_err_to_name(err));
      }
    }
  }
#endif
}

// =====================================================================
//                    ESP-NOW CALLBACKS (DISPATCH)
// =====================================================================

// Promiscuous sniffer callback for software CCA. Only ESP-NOW frames
// (vendor-specific action frames) from known WSN peers count as "busy";
// foreign Wi-Fi traffic is arbitrated by the hardware 802.11 DCF.
void promisc_rx_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT || buf == nullptr) return;
  const wifi_promiscuous_pkt_t* p = (const wifi_promiscuous_pkt_t*)buf;
  if (p->rx_ctrl.sig_len < 25) return;
  const uint8_t* f = p->payload;
  uint8_t subtype = (f[0] >> 4) & 0x0F;
  if (subtype != 0x0D || f[24] != 127) return;   // action frame, vendor-specific category
  const uint8_t* sa = f + 10;                     // addr2 = transmitter
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    if (memcmp(sa, mesh.peer_macs[i], 6) == 0 || memcmp(sa, NODE_MAC[i], 6) == 0) {
      mesh.last_rx_time_ms = millis();
      return;
    }
  }
}

// --- OnDataRecv: validation + master dispatcher ---
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void mesh_on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* src_mac = (info) ? info->src_addr : nullptr;
  int rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : -50;
#else
void mesh_on_recv(const uint8_t* mac_addr, const uint8_t* data, int len) {
  const uint8_t* src_mac = mac_addr;
  int rssi = -50;
#endif
  if (len < WSN_HEADER_LEN || len > (int)sizeof(wsn_packet_t)) return;

  wsn_packet_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  memcpy(&pkt, data, len);

  // Ignore foreign ESP-NOW traffic
  if (pkt.magic != WSN_MAGIC) return;
  if (pkt.src_id < 1 || pkt.src_id > NUM_NODES || pkt.src_id == MY_NODE_ID) return;

  // Structural validation
  int min_len = wsn_min_payload_len(pkt.msg_type);
  if (min_len < 0 || pkt.payload_len != len - WSN_HEADER_LEN || pkt.payload_len < min_len ||
      pkt.origin_id < 1 || pkt.origin_id > NUM_NODES ||
      (pkt.dst_id != BROADCAST_ID && (pkt.dst_id < 1 || pkt.dst_id > NUM_NODES))) {
    mesh.mac_stats.malformed_rx++;
    if (mesh.mac_stats.malformed_rx % 10 == 1) {
      LOGE("[MESH] Malformed frame type=0x%02X from N%d (len=%d payload_len=%d) — dropped\n",
                    pkt.msg_type, pkt.src_id, len, pkt.payload_len);
    }
    return;
  }

  // Dynamic peer MAC auto-learning
  if (src_mac && memcmp(mesh.peer_macs[pkt.src_id], src_mac, 6) != 0) {
    memcpy(mesh.peer_macs[pkt.src_id], src_mac, 6);
    if (!esp_now_is_peer_exist(src_mac)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, src_mac, 6);
      peer.channel = 0;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
    Serial.printf("[MESH] Auto-learned MAC for N%d: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  pkt.src_id, src_mac[0], src_mac[1], src_mac[2],
                  src_mac[3], src_mac[4], src_mac[5]);
  }

  mesh.last_rx_time_ms = millis();

  // Control commands are management traffic: processed even on a
  // software-killed link, otherwise RESTORE could never reach that node.
  if (pkt.msg_type == MSG_CTRL) {
    mesh_handle_ctrl(pkt);
    return;
  }

  // Software link kill (failure INJECTION): silently drop
  if (mesh.killed_links[pkt.src_id]) return;
  mesh.last_heard_ms[pkt.src_id] = millis();

  switch (pkt.msg_type) {
    case MSG_ACK:          mesh_handle_ack(pkt); break;
    case MSG_HELLO:        mesh_handle_hello(pkt, rssi); break;
    case MSG_BEACON:       mesh_handle_beacon(pkt, rssi); break;
    case MSG_SCHEDULE:     mesh_handle_schedule(pkt); break;
    case MSG_DATA:         mesh_handle_data(pkt, rssi); break;
    case MSG_AGG:          mesh_handle_agg(pkt, rssi); break;
    case MSG_ROUTE_UPDATE: mesh_handle_route_update(pkt); break;
    case MSG_BLE_REPORT:   mesh_handle_ble_report(pkt, rssi); break;
    default: break;
  }
}

// --- OnDataSent callback ---
// ESP-IDF 5.5 (Arduino-ESP32 3.3.x) changed the send-callback signature.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void mesh_on_sent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
#else
void mesh_on_sent(const uint8_t* mac, esp_now_send_status_t status) {
#endif
  // Link-layer (802.11) delivery status; end-to-end hop reliability uses the app ACK.
  // Failures are normal under interference (the app-level ACK/retransmit
  // recovers them), so they are only counted (MAC line) and traced at log 2.
  if (status != ESP_NOW_SEND_SUCCESS) {
    mesh.mac_stats.ll_fail++;
    LOGV("[ESP-NOW] Link-layer send failed (total %lu)\n", (unsigned long)mesh.mac_stats.ll_fail);
  }
}

// =====================================================================
//                    MESSAGE HANDLERS
// =====================================================================

void mesh_handle_ack(const wsn_packet_t& pkt) {
  if (pkt.dst_id != MY_NODE_ID) return;
  ack_payload_t ap;
  getPayload(pkt, ap);
  if (mesh.ack_pending &&
      pkt.src_id == mesh.ack_expected_from &&
      ap.acked_seq_num == mesh.ack_expected_seq &&
      ap.acked_origin_id == mesh.ack_expected_origin &&
      ap.acked_type == mesh.ack_expected_type) {
    mesh.ack_received = true;
  }
}

// Common unicast receive path: ACK, duplicate check, log
bool mesh_accept_unicast(const wsn_packet_t& pkt) {
  if (pkt.dst_id != MY_NODE_ID) return false;   // not addressed to us: no ACK, no processing
  mac_send_ack(pkt);
  if (mesh_seen_before(pkt.msg_type, pkt.origin_id, pkt.seq_num)) {
    mesh.mac_stats.dup_rx++;
    mesh.dup_rx_by_origin[pkt.origin_id]++;
    LOGV("[MAC] Duplicate %s seq=%d origin=N%d from N%d (re-ACKed, not re-processed)\n",
                  msgTypeStr(pkt.msg_type), pkt.seq_num, pkt.origin_id, pkt.src_id);
    return false;
  }
  logPacket("RX", pkt);
  return true;
}

void mesh_handle_data(const wsn_packet_t& pkt, int rssi) {
  if (!mesh_accept_unicast(pkt)) return;
  if (pkt.final_dst_id == MY_NODE_ID) {
    mesh.pkts_received_at_sink++;
    app_on_data_for_me(pkt, rssi);
  } else if (!app_on_data_transit(pkt, rssi)) {
    mesh_enqueue_forward(pkt);
  }
}

void mesh_handle_agg(const wsn_packet_t& pkt, int rssi) {
  if (!mesh_accept_unicast(pkt)) return;
  if (pkt.final_dst_id == MY_NODE_ID) {
    mesh.pkts_received_at_sink++;
    app_on_data_for_me(pkt, rssi);
  } else {
    mesh_enqueue_forward(pkt);
  }
}

// BLE reports travel multi-hop to the sink like data (forwarded, not consumed)
void mesh_handle_ble_report(const wsn_packet_t& pkt, int rssi) {
  if (!mesh_accept_unicast(pkt)) return;
  if (pkt.final_dst_id == MY_NODE_ID) {
    app_on_data_for_me(pkt, rssi);
  } else {
    mesh_enqueue_forward(pkt);
  }
}

void mesh_handle_ctrl(const wsn_packet_t& pkt) {
  if (mesh_seen_before(MSG_CTRL, pkt.origin_id, pkt.seq_num)) return;  // N4 repeats CTRL 3x
  ctrl_payload_t cp;
  getPayload(pkt, cp);
  logPacket("RX", pkt);

  switch (cp.cmd) {
    case CTRL_COLLISION_TEST:
      mesh.collision_test_trigger = true;
      wsn_log_force_until = millis() + 3000;   // show the full CSMA trace of the contention
      LOGE("[CTRL] Collision test armed (N1/N2 will contend at the start of the next CAP)\n");
      break;

    case CTRL_KILL_LINK:
      if (cp.param1 == MY_NODE_ID || cp.param2 == MY_NODE_ID) {
        uint8_t other = (cp.param1 == MY_NODE_ID) ? cp.param2 : cp.param1;
        if (other >= 1 && other <= NUM_NODES) {
          LOGE("[CTRL] Software link kill (failure injection): N%d↔N%d\n", cp.param1, cp.param2);
          routing_mark_link_dead(other, "software link kill");
          mesh.killed_links[other] = true;
          routing_recompute_next_hop();
        }
      }
      break;

    case CTRL_RESTORE_LINKS:
      for (int i = 1; i <= NUM_NODES; i++) mesh.killed_links[i] = false;
      LOGE("[CTRL] All software link kills restored (links come back on next HELLO/BEACON)\n");
      mesh.hello_triggered = true;
      routing_recompute_next_hop();
      break;
  }

  app_on_ctrl((ctrl_cmd_t)cp.cmd, cp.param1, cp.param2);
}

// =====================================================================
//                    MESH INIT & PERIODIC LOOP
// =====================================================================

void mesh_init() {
  memset(&mesh, 0, sizeof(mesh));
  mesh.my_id = MY_NODE_ID;
  mesh.my_role = NODE_ROLES[MY_NODE_ID];
  mesh.current_cw = CW_MIN;
  mesh.next_hop_to_sink = 0;
  mesh.data_seq_counter = 1;
  mesh.ctrl_seq_counter = 1;
  randomSeed(esp_random());

  routing_init();
  tdma_init();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[MESH] ESP-NOW init FAILED!");
    return;
  }

  esp_now_register_recv_cb(mesh_on_recv);
  esp_now_register_send_cb(mesh_on_sent);

  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, NODE_MAC[i], 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.printf("[MESH] Failed to add peer N%d\n", i);
    }
  }

  esp_now_peer_info_t bcast = {};
  memcpy(bcast.peer_addr, BROADCAST_MAC, 6);
  bcast.channel = 0;
  bcast.encrypt = false;
  esp_now_add_peer(&bcast);

  // Promiscuous sniffing (management frames only) for software CCA
  wifi_promiscuous_filter_t filt = {};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(promisc_rx_cb);
  esp_wifi_set_promiscuous(true);

  uint8_t ch = 0;
  wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("[FW] %s | log level %d (type 'log 2' for the full packet trace)\n", WSN_FW_VERSION, wsn_log_level);
  Serial.printf("[MESH] Initialized: N%d (%s) | ESP-NOW on Wi-Fi channel %d\n",
                MY_NODE_ID, roleStr(mesh.my_role), ch);
  Serial.printf("[MESH] My MAC: %s\n", WiFi.macAddress().c_str());
}

void mesh_loop() {
  uint32_t now = millis();

  tdma_tick();
  tdma_epoch_t epoch = tdma_current_epoch();

  // 0. Synchronisation maintenance (N1–N3)
  if (MY_NODE_ID != SINK_ID) {
    if (mesh.tdma.synced && wsn_age_ms(mesh.tdma.last_sync_ms) > SYNC_LOST_MS) {
      mesh.tdma.synced = false;
      mesh.tdma.synced_from_beacon = false;
      LOGE("[TDMA] *** Sync lost (no beacon / synced HELLO for %lu ms) — rescanning channels ***\n",
                    (unsigned long)wsn_age_ms(mesh.tdma.last_sync_ms));
    }
    if (!mesh.tdma.synced) {
      static uint32_t lastChannelHop = 0;
      static uint8_t currentChannel = 1;
      if (now - lastChannelHop >= 1200) {
        lastChannelHop = now;
        currentChannel++;
        if (currentChannel > 13) currentChannel = 1;
        esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
        Serial.printf("[MESH] Scanning Wi-Fi Channel %d for N4 beacon...\n", currentChannel);
      }
    }
  }

  // 1. N4: beacon at the start of every superframe
  if (MY_NODE_ID == SINK_ID && epoch == EPOCH_BEACON) {
    static uint32_t last_beacon_frame = 0xFFFFFFFF;
    uint32_t frame = tdma_current_frame();
    if (frame != last_beacon_frame) {
      last_beacon_frame = frame;
      tdma_send_beacon();
    }
  }

  // 2. Control traffic in the CAP (or any time while unsynced)
  if (mesh_ctrl_window_open() && tdma_epoch_remaining_ms() > 10) {
    if (mesh.route_update_pending) {
      mesh.route_update_pending = false;
      wsn_packet_t rupkt;
      initPacket(rupkt, MSG_ROUTE_UPDATE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, mesh.ctrl_seq_counter++);
      setPayload(rupkt, mesh.route_update_payload);
      mac_send_broadcast(rupkt, true);
    }
    if (mesh.sched_is_coordinator &&
        (mesh.sched_announce_pending || now - mesh.last_sched_announce_ms >= SCHED_REBROADCAST_MS)) {
      bool changed = mesh.sched_announce_pending;
      mesh.sched_announce_pending = false;
      tdma_announce_schedule(changed);
    }
    // N4: relay the current N3 schedule (nodes out of N3's radio range get it too)
    if (MY_NODE_ID == SINK_ID && !mesh.sched_is_coordinator && mesh.last_primary_rx_ms != 0 &&
        wsn_age_ms(mesh.last_primary_rx_ms) < 2 * SCHED_REBROADCAST_MS &&
        now - mesh.last_sched_relay_ms >= SCHED_REBROADCAST_MS) {
      mesh.last_sched_relay_ms = now;
      wsn_packet_t rp;
      initPacket(rp, MSG_SCHEDULE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, 0);
      setPayload(rp, mesh.last_primary_sched);
      mac_send_broadcast(rp, false);
      LOGV("[TDMA] Relayed N3 schedule v%u\n", mesh.last_primary_sched.version);
    }
    if ((mesh.hello_triggered && now - mesh.last_hello_ms >= HELLO_MIN_GAP_MS) ||
        now - mesh.last_hello_ms >= HELLO_INTERVAL_MS) {
      routing_send_hello();
    }
  }

  // 3. Neighbour liveness
  routing_check_timeouts();

  // 4. Forward queued transit packets (relay slot or CAP)
  mesh_service_forward_queue();
}

// =====================================================================
//            OWN PACKETS: build, buffer, (re)transmit
// =====================================================================

bool mesh_own_tx_busy(uint8_t idx) { return mesh.own_tx[idx].active; }

void mesh_own_tx_load(uint8_t idx, const wsn_packet_t& pkt) {
  mesh.own_tx[idx].pkt = pkt;
  mesh.own_tx[idx].attempts = 0;
  mesh.own_tx[idx].defer_key = 0xFFFFFFFF;
  mesh.own_tx[idx].active = true;
}

// Transmit (or continue transmitting) a buffered own packet in the current window
mac_result_t mesh_own_tx_service(uint8_t idx) {
  own_tx_t& o = mesh.own_tx[idx];
  if (!o.active) return MAC_OK;
  if (!mesh_retry_window_open()) return MAC_DEFERRED;   // never transmit outside own slot / CAP
  uint32_t key = mesh_tx_opportunity_key();
  if (o.defer_key == key) return MAC_DEFERRED;   // wait for the next slot/CAP
  uint8_t next = mesh.next_hop_to_sink;
  if (next == 0) return MAC_NO_ROUTE;      // keep it until a route exists
  o.pkt.dst_id = next;                     // route may have changed since creation
  mac_result_t r = mac_send(o.pkt, o.attempts, tdma_epoch_deadline());
  if (r == MAC_OK || r == MAC_DROPPED) o.active = false;
  if (r == MAC_DEFERRED) o.defer_key = key;
  return r;
}

wsn_packet_t mesh_build_sensor_packet(float v1, float v2, float v3, const char* sensor_type) {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_DATA, mesh.next_hop_to_sink, MY_NODE_ID, SINK_ID, mesh.data_seq_counter++);
  sensor_payload_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.val1 = v1;
  sp.val2 = v2;
  sp.val3 = v3;
  strncpy(sp.sensor_type, sensor_type, sizeof(sp.sensor_type) - 1);
  setPayload(pkt, sp);
  return pkt;
}

wsn_packet_t mesh_build_agg_packet(const agg_payload_t& agg) {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_AGG, mesh.next_hop_to_sink, MY_NODE_ID, SINK_ID, mesh.data_seq_counter++);
  setPayload(pkt, agg);
  return pkt;
}

wsn_packet_t mesh_build_ble_report_packet(const ble_report_payload_t& rpt) {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_BLE_REPORT, mesh.next_hop_to_sink, MY_NODE_ID, SINK_ID, mesh.ctrl_seq_counter++);
  setPayload(pkt, rpt);
  return pkt;
}

// =====================================================================
//           CONTENTION DEMO: Collision test sender (N1 and N2)
// =====================================================================
// After CTRL_COLLISION_TEST, N1 and N2 both wait for the start of the next
// CAP and then transmit to their next hop at the same moment. CCA + random
// backoff decide who goes first; the loser senses the winner's frame and
// defers (log: "[MAC] Contention: channel taken during backoff").
void mesh_collision_test_service() {
  if (!mesh.collision_test_trigger) return;
  if (MY_NODE_ID != 1 && MY_NODE_ID != 2) { mesh.collision_test_trigger = false; return; }
  if (mesh.tdma.synced && (tdma_current_epoch() != EPOCH_CAP || tdma_epoch_remaining_ms() < CAP_DURATION_MS - 20)) return;
  if (mesh.next_hop_to_sink == 0) return;
  mesh.collision_test_trigger = false;

  Serial.printf("[COLLISION TEST] CAP start (synced t=%lu) — contending for the channel now\n",
                (unsigned long)(tdma_synced_time() % SUPERFRAME_MS));
  // Use the normal own-packet buffer so a deferred/lost test frame is retried
  if (!mesh_own_tx_busy(OWN_TX_DATA)) {
    mesh_own_tx_load(OWN_TX_DATA, mesh_build_sensor_packet(0, 0, 0, "colltest"));
  }
  mac_result_t r = mesh_own_tx_service(OWN_TX_DATA);
  Serial.printf("[COLLISION TEST] Result: %s (attempts=%d), backoffs so far=%lu, CCA busy so far=%lu\n",
                r == MAC_OK ? "DELIVERED (ACK)" : (r == MAC_DEFERRED ? "DEFERRED to next slot/CAP" : "FAILED"),
                mesh.own_tx[OWN_TX_DATA].attempts, (unsigned long)mesh.mac_stats.total_backoffs,
                (unsigned long)mesh.mac_stats.cca_busy_count);
}

// =====================================================================
//                         STATUS
// =====================================================================
void mesh_print_status() {
  Serial.println("\n====== MESH STATUS ======");
  Serial.printf("Node: N%d (%s) | next hop to sink: %s (cost=%.2f, hops=%d)\n",
                MY_NODE_ID, roleStr(mesh.my_role),
                mesh.next_hop_to_sink ? nodeIdStr(mesh.next_hop_to_sink) : "NONE",
                mesh.next_hop_cost, routing_my_hops());
  Serial.printf("TDMA synced: %s (%s) | Frame: %lu | Schedule v%u from N%d (%s)\n",
    mesh.tdma.synced ? "YES" : "NO", mesh.tdma.synced_from_beacon ? "beacon" : "neighbour",
    (unsigned long)tdma_current_frame(), mesh.tdma.sched_version, mesh.tdma.sched_issuer, mesh.tdma.sched_reason);

  uint32_t total_time = mesh.tdma.active_time_ms + mesh.tdma.sleep_time_ms;
  float duty_cycle = (total_time > 0) ? (mesh.tdma.active_time_ms * 100.0f / total_time) : 100.0f;
  Serial.printf("Duty cycle: %.1f%% (active=%lu ms, idle/sleep=%lu ms) | unused own slots=%lu\n",
    duty_cycle, (unsigned long)mesh.tdma.active_time_ms, (unsigned long)mesh.tdma.sleep_time_ms,
    (unsigned long)mesh.tdma.unused_slots);

  Serial.println("--- Neighbour / routing table ---");
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    route_neighbor_t& nb = mesh.neighbors[i];
    float c = routing_cost(i);
    char cost[12];
    if (c >= ROUTE_INFINITY) snprintf(cost, sizeof(cost), "inf");
    else snprintf(cost, sizeof(cost), "%.2f", c);
    Serial.printf("  N%d: %s | RSSI=%d | ACK=%.0f%% | hops=%d | via=%s | cost=%s | kill=%s | retries=%u\n",
      i, nb.link_alive ? "ALIVE" : "DEAD",
      nb.wifi_rssi, nb.ack_success_rate * 100,
      nb.hops_to_sink, nb.next_hop ? nodeIdStr(nb.next_hop) : "-", cost,
      mesh.killed_links[i] ? "YES" : "NO", nb.reported_retries);
  }
  Serial.printf("--- MAC Stats ---\n");
  Serial.printf("  CCA busy=%lu backoffs=%lu tx_attempts=%lu retransmissions=%lu drops=%lu ACKs=%lu timeouts=%lu deferrals=%lu reroutes=%lu dup_rx=%lu malformed=%lu\n",
    (unsigned long)mesh.mac_stats.cca_busy_count, (unsigned long)mesh.mac_stats.total_backoffs,
    (unsigned long)mesh.mac_stats.total_tx_attempts, (unsigned long)mesh.mac_stats.total_retries,
    (unsigned long)mesh.mac_stats.total_drops, (unsigned long)mesh.mac_stats.total_acks_ok,
    (unsigned long)mesh.mac_stats.total_acks_timeout, (unsigned long)mesh.mac_stats.total_deferrals,
    (unsigned long)mesh.mac_stats.total_reroutes, (unsigned long)mesh.mac_stats.dup_rx,
    (unsigned long)mesh.mac_stats.malformed_rx);
  Serial.printf("  Sent=%lu Fwd=%lu FwdQueue=%d%s\n",
    (unsigned long)mesh.pkts_sent, (unsigned long)mesh.pkts_forwarded, mesh.fwd_queue.count,
    MY_NODE_ID == SINK_ID ? "" : "");
  Serial.println("=========================\n");
}

#endif // WSN_MESH_H
