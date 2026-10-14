// ==================== WSN MESH PROTOCOL STACK ====================
// Full mesh stack: MAC (CCA + BEB + ACK), Routing (neighbor table +
// distance-vector + forwarding), TDMA (sync + slot engine + sleep).
//
// This file is header-only. Include it after wsn_common.h.
// Every node compiles the same code; role-specific behavior is
// controlled by MY_NODE_ID and NODE_ROLES[MY_NODE_ID].
// =================================================================
#ifndef WSN_MESH_H
#define WSN_MESH_H

#include "wsn_common.h"

// ==================== GLOBAL MESH STATE ====================
// Single instance per node (one compilation unit per Arduino sketch)
mesh_state_t mesh;

// ==================== FORWARD DECLARATIONS ====================
void mesh_handle_data(const wsn_packet_t& pkt, int rssi);
void mesh_handle_ack(const wsn_packet_t& pkt);
void mesh_handle_hello(const wsn_packet_t& pkt, int rssi);
void mesh_handle_beacon(const wsn_packet_t& pkt);
void mesh_handle_schedule(const wsn_packet_t& pkt);
void mesh_handle_route_update(const wsn_packet_t& pkt);
void mesh_handle_ctrl(const wsn_packet_t& pkt);
void mesh_handle_ble_report(const wsn_packet_t& pkt);
void mesh_handle_agg(const wsn_packet_t& pkt, int rssi);

// Routing & MAC layer forward declarations
void routing_recompute_next_hop();
void routing_send_hello();
bool routing_forward(wsn_packet_t pkt);
void routing_check_timeouts();
bool mac_send(wsn_packet_t& pkt);
void mac_send_ack(const wsn_packet_t& received_pkt);
bool espnow_raw_send(uint8_t dst_id, const wsn_packet_t& pkt);

// Application-level callbacks (implemented in each node's .ino)
extern void app_on_data_for_me(const wsn_packet_t& pkt, int rssi);
extern void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2);
// Called when this node is an intermediate hop for a DATA packet.
// Return TRUE to consume the packet (e.g., N3 buffers it for aggregation).
// Return FALSE to let the mesh layer forward it normally.
extern bool app_on_data_transit(const wsn_packet_t& pkt, int rssi);

// ==================== ESP-NOW RAW SEND HELPER ====================
// Low-level send — no CCA, no ACK, no retry. Used internally.
bool espnow_raw_send(uint8_t dst_id, const wsn_packet_t& pkt) {
  const uint8_t* mac;
  if (dst_id == BROADCAST_ID) {
    mac = BROADCAST_MAC;
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

  esp_err_t result = esp_now_send(mac, (const uint8_t*)&pkt, sizeof(wsn_packet_t));
  if (result != ESP_OK) {
    Serial.printf("[ESP-NOW] Raw send FAILED to %s (err=%d)\n", nodeIdStr(dst_id), result);
    return false;
  }
  return true;
}

// =====================================================================
//                          MAC LAYER
// =====================================================================

// --- Software Clear Channel Assessment (CCA) ---
// Returns true if channel appears idle.
bool mac_cca() {
  if (millis() - mesh.last_rx_time_ms < CCA_BUSY_THRESHOLD_MS) {
    mesh.mac_stats.cca_busy_count++;
    Serial.printf("[MAC] CCA: BUSY (frame seen %lu ms ago)\n",
                  millis() - mesh.last_rx_time_ms);
    return false;
  }
  return true;
}

// --- Binary Exponential Backoff ---
void mac_backoff() {
  uint8_t slots = random(0, mesh.current_cw);
  uint32_t wait_ms = slots * BACKOFF_SLOT_MS;
  mesh.mac_stats.total_backoffs++;
  Serial.printf("[MAC] Backoff: CW=%d slots=%d wait=%lu ms\n",
                mesh.current_cw, slots, wait_ms);
  delay(wait_ms);
  // Double the contention window
  mesh.current_cw = min((int)(mesh.current_cw * 2), (int)CW_MAX);
}

// --- Send with CSMA/CA + Application-Level ACK ---
// Returns true if ACK received (unicast) or send succeeded (broadcast).
// This function is BLOCKING during the send+ACK window.
bool mac_send(wsn_packet_t& pkt) {
  bool is_broadcast = (pkt.dst_id == BROADCAST_ID);
  mesh.current_cw = CW_MIN;

  for (int attempt = 0; attempt <= MAX_RETRIES; attempt++) {
    // 1. Clear Channel Assessment
    if (!mac_cca()) {
      mac_backoff();
      // Re-check CCA after backoff
      if (!mac_cca()) {
        // Still busy — count as retry if unicast
        if (!is_broadcast) {
          mesh.mac_stats.total_retries++;
          Serial.printf("[MAC] CCA still busy after backoff (attempt %d/%d)\n",
                        attempt + 1, MAX_RETRIES + 1);
          continue;
        }
      }
    }

    // 2. Set up ACK expectation (unicast only)
    if (!is_broadcast) {
      mesh.ack_pending = true;
      mesh.ack_expected_seq = pkt.seq_num;
      mesh.ack_expected_origin = pkt.origin_id;
      mesh.ack_received = false;
    }

    // 3. Transmit
    pkt.src_id = MY_NODE_ID;  // Always set src to us (for forwarded packets)
    logPacket("TX", pkt);
    bool sent = espnow_raw_send(pkt.dst_id, pkt);
    mesh.pkts_sent++;

    if (!sent) {
      mesh.ack_pending = false;
      if (!is_broadcast) {
        mesh.mac_stats.total_retries++;
        mac_backoff();
        continue;
      }
      return false;
    }

    // 4. For broadcast: no ACK needed, done
    if (is_broadcast) {
      return true;
    }

    // 5. Wait for application-level ACK
    uint32_t ack_start = millis();
    while (!mesh.ack_received && (millis() - ack_start) < ACK_TIMEOUT_MS) {
      delayMicroseconds(200);
    }
    mesh.ack_pending = false;

    if (mesh.ack_received) {
      // ACK received — success!
      uint32_t rtt = millis() - ack_start;
      mesh.mac_stats.total_acks_ok++;
      mesh.current_cw = CW_MIN;  // Reset CW on success

      // Update neighbor ACK stats
      uint8_t nb = pkt.dst_id;
      if (nb >= 1 && nb <= NUM_NODES) {
        mesh.neighbors[nb].ack_sent++;
        mesh.neighbors[nb].ack_received++;
        mesh.neighbors[nb].consec_ack_fails = 0;
        // EWMA ACK success rate
        float rate = (float)mesh.neighbors[nb].ack_received / mesh.neighbors[nb].ack_sent;
        mesh.neighbors[nb].ack_success_rate =
          0.3f * rate + 0.7f * mesh.neighbors[nb].ack_success_rate;
      }

      Serial.printf("[MAC] ACK OK for seq=%d from N%d (RTT=%lu ms)\n",
                    pkt.seq_num, pkt.dst_id, rtt);
      return true;
    }

    // 6. ACK timeout — retry
    mesh.mac_stats.total_acks_timeout++;
    mesh.mac_stats.total_retries++;
    Serial.printf("[MAC] ACK TIMEOUT seq=%d → retry %d/%d\n",
                  pkt.seq_num, attempt + 1, MAX_RETRIES);

    // Update neighbor failure stats (EWMA)
    uint8_t nb2 = pkt.dst_id;
    if (nb2 >= 1 && nb2 <= NUM_NODES) {
      mesh.neighbors[nb2].ack_sent++;
      float rate = (float)mesh.neighbors[nb2].ack_received / mesh.neighbors[nb2].ack_sent;
      mesh.neighbors[nb2].ack_success_rate =
        0.3f * rate + 0.7f * mesh.neighbors[nb2].ack_success_rate;
    }

    mac_backoff();
  }

  // All retries exhausted — report packet dropped
  mesh.mac_stats.total_drops++;
  uint8_t nb3 = pkt.dst_id;
  Serial.printf("[MAC] DROPPED seq=%d to N%d after %d retries\n",
                pkt.seq_num, nb3, MAX_RETRIES + 1);

  // Check if this neighbor should be marked dead (count DROPPED packets, not retries)
  if (nb3 >= 1 && nb3 <= NUM_NODES) {
    mesh.neighbors[nb3].consec_ack_fails++;
    if (mesh.neighbors[nb3].consec_ack_fails >= LINK_FAIL_THRESHOLD) {
      if (mesh.neighbors[nb3].link_alive) {
        mesh.neighbors[nb3].link_alive = false;
        Serial.printf("[ROUTE] *** Link N%d→N%d DEAD (%d consecutive dropped packets) ***\n",
                      MY_NODE_ID, nb3, mesh.neighbors[nb3].consec_ack_fails);

        // Broadcast route update
        wsn_packet_t rupkt;
        initPacket(rupkt, MSG_ROUTE_UPDATE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, mesh.ctrl_seq_counter++);
        route_update_payload_t ru = { MY_NODE_ID, nb3, false };
        setPayload(rupkt, ru);
        espnow_raw_send(BROADCAST_ID, rupkt);

        // Recompute routes
        routing_recompute_next_hop();
      }
    }
  }

  return false;
}

// --- Send an ACK for a received unicast packet ---
void mac_send_ack(const wsn_packet_t& received_pkt) {
  wsn_packet_t ack;
  initPacket(ack, MSG_ACK, received_pkt.src_id, MY_NODE_ID, received_pkt.src_id, 0);
  ack_payload_t ap;
  ap.acked_seq_num = received_pkt.seq_num;
  ap.acked_origin_id = received_pkt.origin_id;
  setPayload(ack, ap);
  // ACKs are sent raw (no CCA/retry to avoid recursion)
  espnow_raw_send(received_pkt.src_id, ack);
}

// =====================================================================
//                        ROUTING LAYER
// =====================================================================

// Forward declaration (used by routing_recompute)
void routing_recompute_next_hop();

// --- Initialize neighbor table ---
void routing_init() {
  for (int i = 0; i <= NUM_NODES; i++) {
    mesh.neighbors[i].node_id = i;
    mesh.neighbors[i].wifi_rssi = -100;
    mesh.neighbors[i].ack_success_rate = 0.5f;
    mesh.neighbors[i].hops_to_sink = 255;
    mesh.neighbors[i].last_hello_ms = 0;
    mesh.neighbors[i].link_alive = false;
    mesh.neighbors[i].ack_sent = 0;
    mesh.neighbors[i].ack_received = 0;
    mesh.neighbors[i].consec_ack_fails = 0;
  }
  // Sink knows it's 0 hops from itself
  if (MY_NODE_ID == 4) {
    mesh.neighbors[4].hops_to_sink = 0;
    mesh.neighbors[4].link_alive = true;
    mesh.next_hop_to_sink = 4;  // We ARE the sink
  }
}

// --- Compute route cost to reach sink through neighbor i ---
float routing_cost(uint8_t neighbor_id) {
  route_neighbor_t& n = mesh.neighbors[neighbor_id];
  if (!n.link_alive || n.hops_to_sink == 255) return 9999.0f;
  float etx = (n.ack_success_rate > 0.01f) ? (1.0f / n.ack_success_rate) : 100.0f;
  float rssi_norm = (float)(-n.wifi_rssi) / 100.0f;
  float cost = ROUTE_COST_W_HOP * (n.hops_to_sink + 1) +
               ROUTE_COST_W_ETX * etx +
               ROUTE_COST_W_RSSI * rssi_norm;

  // Cluster mesh preference (Defect 2 fix):
  // N1 & N2 prefer routing through Cluster Head N3 for aggregation.
  // Direct link to N4 has +10.0 penalty while N3 is alive.
  // If N3 fails, penalty disappears and nodes immediately failover directly to N4!
  if ((MY_NODE_ID == 1 || MY_NODE_ID == 2) && neighbor_id == 4 && mesh.neighbors[3].link_alive) {
    cost += 10.0f;
  }
  return cost;
}

// --- Recompute best next hop toward the sink ---
void routing_recompute_next_hop() {
  if (MY_NODE_ID == 4) {
    mesh.next_hop_to_sink = 4;
    return;
  }

  float best_cost = 9999.0f;
  uint8_t best_hop = 0;

  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    if (mesh.killed_links[i]) continue;  // Software-killed link
    float c = routing_cost(i);
    if (c < best_cost) {
      best_cost = c;
      best_hop = i;
    }
  }

  if (best_hop != mesh.next_hop_to_sink && best_hop != 0) {
    Serial.printf("[ROUTE] Next hop changed: N%d → N%d (cost=%.2f)\n",
                  mesh.next_hop_to_sink, best_hop, best_cost);
    mesh.next_hop_to_sink = best_hop;
  } else if (best_hop == 0) {
    Serial.println("[ROUTE] WARNING: No route to sink!");
    mesh.next_hop_to_sink = 0;
  }
}

// --- Build and broadcast a HELLO packet ---
void routing_send_hello() {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_HELLO, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, 0);

  hello_payload_t hp;
  hp.node_id = MY_NODE_ID;
  // Compute my hops_to_sink
  if (MY_NODE_ID == 4) {
    hp.hops_to_sink = 0;
  } else if (mesh.next_hop_to_sink != 0) {
    hp.hops_to_sink = mesh.neighbors[mesh.next_hop_to_sink].hops_to_sink + 1;
  } else {
    hp.hops_to_sink = 255;
  }
  hp.ack_success_rate = (mesh.next_hop_to_sink != 0)
    ? mesh.neighbors[mesh.next_hop_to_sink].ack_success_rate : 0.0f;
  hp.wifi_rssi = (mesh.next_hop_to_sink != 0)
    ? mesh.neighbors[mesh.next_hop_to_sink].wifi_rssi : -100;
  hp.node_retries = (uint16_t)mesh.mac_stats.total_retries;

  setPayload(pkt, hp);
  espnow_raw_send(BROADCAST_ID, pkt);  // Broadcast HELLOs without ACK
}

// --- Process incoming HELLO ---
void mesh_handle_hello(const wsn_packet_t& pkt, int rssi) {
  hello_payload_t hp;
  getPayload(pkt, hp);
  uint8_t nid = hp.node_id;
  if (nid < 1 || nid > NUM_NODES || nid == MY_NODE_ID) return;

  // Skip if software-killed
  if (mesh.killed_links[nid]) return;

  route_neighbor_t& nb = mesh.neighbors[nid];
  bool was_dead = !nb.link_alive;
  nb.link_alive = true;
  nb.consec_ack_fails = 0;  // Reset failure count upon hearing neighbor
  nb.last_hello_ms = millis();
  nb.wifi_rssi = rssi;
  nb.hops_to_sink = hp.hops_to_sink;
  nb.reported_retries = hp.node_retries;
  // Don't overwrite local ACK stats — HELLO reports the *sender's* outbound stats

  if (was_dead) {
    Serial.printf("[ROUTE] Link N%d↔N%d RESTORED (HELLO received, RSSI=%d)\n",
                  MY_NODE_ID, nid, rssi);
    // Broadcast route update
    wsn_packet_t rupkt;
    initPacket(rupkt, MSG_ROUTE_UPDATE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, mesh.ctrl_seq_counter++);
    route_update_payload_t ru = { MY_NODE_ID, nid, true };
    setPayload(rupkt, ru);
    espnow_raw_send(BROADCAST_ID, rupkt);
  }

  routing_recompute_next_hop();
}

// --- Forward a packet toward its final destination ---
bool routing_forward(wsn_packet_t pkt) {
  // Decrement TTL
  if (pkt.ttl == 0) {
    Serial.printf("[ROUTE] Dropping packet seq=%d — TTL expired\n", pkt.seq_num);
    return false;
  }
  pkt.ttl--;
  pkt.hop_count++;

  // Record hop in route_path
  if (pkt.hop_count < 4) {
    pkt.route_path[pkt.hop_count] = MY_NODE_ID;
  }

  // Determine next hop
  uint8_t next = mesh.next_hop_to_sink;
  if (next == 0) {
    Serial.printf("[ROUTE] Cannot forward seq=%d — no route to sink\n", pkt.seq_num);
    return false;
  }

  pkt.dst_id = next;
  Serial.printf("[ROUTE] Forwarding seq=%d (origin=N%d) via N%d\n",
                pkt.seq_num, pkt.origin_id, next);
  mesh.pkts_forwarded++;
  return mac_send(pkt);
}

// --- Check neighbor timeouts ---
void routing_check_timeouts() {
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    route_neighbor_t& nb = mesh.neighbors[i];
    if (nb.link_alive && nb.last_hello_ms > 0 &&
        millis() - nb.last_hello_ms > HELLO_TIMEOUT_MS) {
      nb.link_alive = false;
      Serial.printf("[ROUTE] Link N%d↔N%d DEAD (HELLO timeout: %lu ms)\n",
                    MY_NODE_ID, i, millis() - nb.last_hello_ms);

      // Broadcast route update
      wsn_packet_t rupkt;
      initPacket(rupkt, MSG_ROUTE_UPDATE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, mesh.ctrl_seq_counter++);
      route_update_payload_t ru = { MY_NODE_ID, (uint8_t)i, false };
      setPayload(rupkt, ru);
      espnow_raw_send(BROADCAST_ID, rupkt);

      routing_recompute_next_hop();
    }
  }
}

// --- Handle route update broadcast ---
void mesh_handle_route_update(const wsn_packet_t& pkt) {
  route_update_payload_t ru;
  getPayload(pkt, ru);
  Serial.printf("[ROUTE] Route update: Link N%d↔N%d %s\n",
                ru.node_a, ru.node_b, ru.link_up ? "UP" : "DOWN");
  // Recompute our route (the topology changed)
  routing_recompute_next_hop();
}

// =====================================================================
//                         TDMA LAYER
// =====================================================================

// --- Initialize TDMA state ---
void tdma_init() {
  mesh.tdma.clock_offset = 0;
  mesh.tdma.frame_number = 0;
  mesh.tdma.synced = false;
  mesh.tdma.last_beacon_rx_ms = 0;
  mesh.tdma.my_tx_slot = 0xFF;
  mesh.tdma.my_relay_slot = 0xFF;
  mesh.tdma.my_tx_slots_mask = 0;
  mesh.tdma.my_relay_slots_mask = 0;
  mesh.tdma.schedule_received = false;
  mesh.tdma.active_time_ms = 0;
  mesh.tdma.sleep_time_ms = 0;
  mesh.tdma.last_epoch_switch_ms = millis();
  mesh.tdma.last_epoch = EPOCH_SLEEP;
}

// --- Get synchronized time ---
uint32_t tdma_synced_time() {
  return millis() + mesh.tdma.clock_offset;
}

// --- Get current epoch within superframe ---
tdma_epoch_t tdma_current_epoch() {
  if (!mesh.tdma.synced) return EPOCH_CAP;  // Default to CAP if not synced
  uint32_t t = tdma_synced_time() % SUPERFRAME_MS;
  if (t < BEACON_START_MS + BEACON_DURATION_MS) return EPOCH_BEACON;
  if (t < CAP_START_MS + CAP_DURATION_MS)       return EPOCH_CAP;
  if (t < SLOT1_START_MS + SLOT1_DURATION_MS)   return EPOCH_SLOT1;
  if (t < SLOT2_START_MS + SLOT2_DURATION_MS)   return EPOCH_SLOT2;
  if (t < SLOT3_START_MS + SLOT3_DURATION_MS)   return EPOCH_SLOT3;
  if (t < SLOT4_START_MS + SLOT4_DURATION_MS)   return EPOCH_SLOT4;
  return EPOCH_SLEEP;
}

// --- Time remaining in current epoch (ms) ---
uint32_t tdma_epoch_remaining_ms() {
  if (!mesh.tdma.synced) return 999;
  uint32_t t = tdma_synced_time() % SUPERFRAME_MS;
  uint32_t epoch_end;
  switch (tdma_current_epoch()) {
    case EPOCH_BEACON: epoch_end = BEACON_START_MS + BEACON_DURATION_MS; break;
    case EPOCH_CAP:    epoch_end = CAP_START_MS + CAP_DURATION_MS; break;
    case EPOCH_SLOT1:  epoch_end = SLOT1_START_MS + SLOT1_DURATION_MS; break;
    case EPOCH_SLOT2:  epoch_end = SLOT2_START_MS + SLOT2_DURATION_MS; break;
    case EPOCH_SLOT3:  epoch_end = SLOT3_START_MS + SLOT3_DURATION_MS; break;
    case EPOCH_SLOT4:  epoch_end = SLOT4_START_MS + SLOT4_DURATION_MS; break;
    case EPOCH_SLEEP:  epoch_end = SUPERFRAME_MS; break;
    default:           epoch_end = SUPERFRAME_MS; break;
  }
  return (epoch_end > t) ? (epoch_end - t) : 0;
}

// --- Is this my TX slot? (Transmitting own sensor data) ---
bool tdma_is_my_tx_slot() {
  if (!mesh.tdma.synced) return true;  // Unsynced: free running
  tdma_epoch_t now = tdma_current_epoch();
  return (mesh.tdma.my_tx_slots_mask & (1 << now)) != 0;
}

// --- Is this my RELAY slot? (Forwarding queued transit packets) ---
bool tdma_is_my_relay_slot() {
  if (!mesh.tdma.synced) return true;  // Unsynced: free running
  tdma_epoch_t now = tdma_current_epoch();
  return (mesh.tdma.my_relay_slots_mask & (1 << now)) != 0;
}

// --- Is this the sleep epoch? ---
bool tdma_is_sleep_epoch() {
  return tdma_current_epoch() == EPOCH_SLEEP;
}

// --- Process incoming beacon from N4 ---
void mesh_handle_beacon(const wsn_packet_t& pkt) {
  if (pkt.origin_id != 4) return;  // Only accept beacons from the sink
  beacon_payload_t bp;
  getPayload(pkt, bp);

  // Sync clock
  mesh.tdma.clock_offset = (int32_t)bp.timestamp_ms - (int32_t)millis();
  mesh.tdma.frame_number = bp.frame_number;
  mesh.tdma.synced = true;
  mesh.tdma.last_beacon_rx_ms = millis();

  // Sink is directly heard! Mark N4 alive and 0 hops
  mesh.neighbors[4].link_alive = true;
  mesh.neighbors[4].consec_ack_fails = 0;  // Reset failure count upon hearing sink
  mesh.neighbors[4].last_hello_ms = millis();
  mesh.neighbors[4].hops_to_sink = 0;
  routing_recompute_next_hop();

  uint8_t ch = 0;
  wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("[TDMA] Beacon sync: frame=%lu offset=%ld ms | Channel %d LOCKED\n",
                bp.frame_number, mesh.tdma.clock_offset, ch);
}

// --- N4: Broadcast a time-sync beacon ---
void tdma_send_beacon() {
  if (MY_NODE_ID != 4) return;  // Only the sink sends beacons

  static uint32_t frame_counter = 0;
  wsn_packet_t pkt;
  initPacket(pkt, MSG_BEACON, BROADCAST_ID, 4, BROADCAST_ID, 0);
  beacon_payload_t bp;
  bp.frame_number = frame_counter++;
  bp.timestamp_ms = millis();
  setPayload(pkt, bp);
  espnow_raw_send(BROADCAST_ID, pkt);
  mesh.tdma.frame_number = bp.frame_number;
}

// --- Process incoming TDMA schedule from N3 (Multi-slot bitmask support) ---
void mesh_handle_schedule(const wsn_packet_t& pkt) {
  schedule_payload_t sp;
  getPayload(pkt, sp);

  mesh.tdma.my_tx_slot = 0xFF;
  mesh.tdma.my_relay_slot = 0xFF;
  mesh.tdma.my_tx_slots_mask = 0;
  mesh.tdma.my_relay_slots_mask = 0;
  mesh.tdma.schedule_received = true;

  Serial.printf("[TDMA] Schedule received (%d slots): %s\n", sp.slot_count, sp.update_reason);

  for (int i = 0; i < sp.slot_count && i < 8; i++) {
    if (sp.slots[i].node_id == MY_NODE_ID) {
      if (sp.slots[i].slot_type == SLOT_TX) {
        mesh.tdma.my_tx_slots_mask |= (1 << sp.slots[i].slot_index);
        mesh.tdma.my_tx_slot = sp.slots[i].slot_index;
        Serial.printf("[TDMA]   My TX slot: EPOCH_%d\n", sp.slots[i].slot_index);
      } else if (sp.slots[i].slot_type == SLOT_RELAY) {
        mesh.tdma.my_relay_slots_mask |= (1 << sp.slots[i].slot_index);
        mesh.tdma.my_relay_slot = sp.slots[i].slot_index;
        Serial.printf("[TDMA]   My RELAY slot: EPOCH_%d\n", sp.slots[i].slot_index);
      }
    }
  }
}

// --- N3: Build and broadcast a default TDMA schedule ---
void tdma_send_default_schedule(const char* reason) {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_SCHEDULE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, 0);

  schedule_payload_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.slot_count = 4;

  // Default assignments
  sp.slots[0] = { 1, EPOCH_SLOT1, SLOT_TX };   // N1 transmits in Slot 1
  sp.slots[1] = { 2, EPOCH_SLOT2, SLOT_TX };   // N2 transmits in Slot 2
  sp.slots[2] = { 3, EPOCH_SLOT3, SLOT_TX };   // N3 (aggregated) in Slot 3
  sp.slots[3] = { 2, EPOCH_SLOT4, SLOT_RELAY }; // N2 relay in Slot 4 (standby)

  strncpy(sp.update_reason, reason, sizeof(sp.update_reason) - 1);
  setPayload(pkt, sp);

  espnow_raw_send(BROADCAST_ID, pkt);

  // Also apply locally
  mesh_handle_schedule(pkt);
}

// --- N3: Dynamic schedule update ---
void tdma_update_schedule_if_needed(bool n1_alert, bool n2_idle, bool n3_offline) {
  if (MY_NODE_ID != 3 && MY_NODE_ID != 4) return;

  wsn_packet_t pkt;
  initPacket(pkt, MSG_SCHEDULE, BROADCAST_ID, MY_NODE_ID, BROADCAST_ID, 0);

  schedule_payload_t sp;
  memset(&sp, 0, sizeof(sp));

  if (n3_offline && MY_NODE_ID == 4) {
    // N4 takes over scheduling when N3 is offline
    sp.slot_count = 3;
    sp.slots[0] = { 1, EPOCH_SLOT1, SLOT_TX };
    sp.slots[1] = { 2, EPOCH_SLOT2, SLOT_TX };
    sp.slots[2] = { 2, EPOCH_SLOT4, SLOT_RELAY };
    strncpy(sp.update_reason, "FAILOVER: N3 offline", sizeof(sp.update_reason) - 1);
  } else if (n1_alert) {
    // N1 alert: give it extra slot
    sp.slot_count = 5;
    sp.slots[0] = { 1, EPOCH_SLOT1, SLOT_TX };
    sp.slots[1] = { 2, EPOCH_SLOT2, SLOT_TX };
    sp.slots[2] = { 3, EPOCH_SLOT3, SLOT_TX };
    sp.slots[3] = { 1, EPOCH_SLOT4, SLOT_TX };  // Extra slot for N1
    sp.slots[4] = { 2, EPOCH_SLOT4, SLOT_SLEEP }; // N2 relay deferred
    strncpy(sp.update_reason, "ALERT: N1 anomaly", sizeof(sp.update_reason) - 1);
  } else if (n2_idle) {
    // N2 idle: release its slot
    sp.slot_count = 3;
    sp.slots[0] = { 1, EPOCH_SLOT1, SLOT_TX };
    sp.slots[1] = { 2, EPOCH_SLOT2, SLOT_SLEEP };
    sp.slots[2] = { 3, EPOCH_SLOT3, SLOT_TX };
    strncpy(sp.update_reason, "IDLE: N2 no data", sizeof(sp.update_reason) - 1);
  } else {
    // Revert back to default schedule when alerts clear! (Defect 4 fix)
    tdma_send_default_schedule("REVERT: normal schedule");
    return;
  }

  setPayload(pkt, sp);
  espnow_raw_send(BROADCAST_ID, pkt);
  mesh_handle_schedule(pkt);
  Serial.printf("[TDMA] Schedule updated: %s\n", sp.update_reason);
}

// =====================================================================
//                    ESP-NOW CALLBACKS (DISPATCH)
// =====================================================================

// Promiscuous sniffer callback for software CCA
void promisc_rx_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  mesh.last_rx_time_ms = millis();
}

// --- OnDataRecv: master dispatcher ---
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void mesh_on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* src_mac = (info) ? info->src_addr : nullptr;
  int rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : -50;
#else
void mesh_on_recv(const uint8_t* mac_addr, const uint8_t* data, int len) {
  const uint8_t* src_mac = mac_addr;
  int rssi = -50;
#endif
  if (len < (int)sizeof(wsn_packet_t)) return;

  wsn_packet_t pkt;
  memcpy(&pkt, data, sizeof(wsn_packet_t));

  // Ignore foreign packets from outside our 4-node network
  if (pkt.src_id < 1 || pkt.src_id > NUM_NODES) return;

  // Dynamic peer MAC auto-learning
  if (src_mac && pkt.src_id >= 1 && pkt.src_id <= NUM_NODES) {
    if (memcmp(mesh.peer_macs[pkt.src_id], src_mac, 6) != 0) {
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
  }

  // Update CCA timestamp
  mesh.last_rx_time_ms = millis();

  // Software link kill check
  if (pkt.src_id >= 1 && pkt.src_id <= NUM_NODES && mesh.killed_links[pkt.src_id]) {
    return;  // Silently drop
  }

  // Dispatch by message type
  switch (pkt.msg_type) {
    case MSG_ACK:
      mesh_handle_ack(pkt);
      break;

    case MSG_HELLO:
      mesh_handle_hello(pkt, rssi);
      break;

    case MSG_BEACON:
      mesh_handle_beacon(pkt);
      break;

    case MSG_SCHEDULE:
      mesh_handle_schedule(pkt);
      break;

    case MSG_DATA:
      mesh_handle_data(pkt, rssi);
      break;

    case MSG_AGG:
      mesh_handle_agg(pkt, rssi);
      break;

    case MSG_ROUTE_UPDATE:
      mesh_handle_route_update(pkt);
      break;

    case MSG_BLE_REPORT:
      mesh_handle_ble_report(pkt);
      break;

    case MSG_CTRL:
      mesh_handle_ctrl(pkt);
      break;

    default:
      Serial.printf("[MESH] Unknown msg_type=0x%02X from N%d\n", pkt.msg_type, pkt.src_id);
      break;
  }
}

// --- OnDataSent callback ---
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void mesh_on_sent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
#else
void mesh_on_sent(const uint8_t* mac, esp_now_send_status_t status) {
#endif
  // Primarily used for debugging; real reliability is via app-level ACK
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Link-layer send failed");
  }
}

// =====================================================================
//                    MESSAGE HANDLERS
// =====================================================================

// --- Handle incoming ACK ---
void mesh_handle_ack(const wsn_packet_t& pkt) {
  if (pkt.dst_id != MY_NODE_ID) return;
  ack_payload_t ap;
  getPayload(pkt, ap);
  if (mesh.ack_pending &&
      ap.acked_seq_num == mesh.ack_expected_seq &&
      ap.acked_origin_id == mesh.ack_expected_origin) {
    mesh.ack_received = true;
  }
}

// --- Handle incoming DATA ---
void mesh_handle_data(const wsn_packet_t& pkt, int rssi) {
  // Send ACK if unicast to us
  if (pkt.dst_id == MY_NODE_ID) {
    mac_send_ack(pkt);
    logPacket("RX", pkt);

    if (pkt.final_dst_id == MY_NODE_ID) {
      // This packet is for ME — deliver to application
      mesh.pkts_received_at_sink++;
      app_on_data_for_me(pkt, rssi);
    } else {
      // Intermediate hop: check if application consumes it (e.g., N3 aggregation)
      if (!app_on_data_transit(pkt, rssi)) {
        // App didn't consume it — queue for forwarding outside callback context (Defect 1 fix)
        if (mesh.fwd_queue.count < FWD_QUEUE_SIZE) {
          mesh.fwd_queue.queue[mesh.fwd_queue.head] = pkt;
          mesh.fwd_queue.head = (mesh.fwd_queue.head + 1) % FWD_QUEUE_SIZE;
          mesh.fwd_queue.count++;
        } else {
          Serial.println("[ROUTE] Fwd queue full — dropping packet");
        }
      }
    }
  }
  // Ignore if not addressed to us (don't ACK, don't process)
}

// --- Handle incoming AGG (same logic as DATA) ---
void mesh_handle_agg(const wsn_packet_t& pkt, int rssi) {
  if (pkt.dst_id == MY_NODE_ID) {
    mac_send_ack(pkt);
    logPacket("RX", pkt);

    if (pkt.final_dst_id == MY_NODE_ID) {
      mesh.pkts_received_at_sink++;
      app_on_data_for_me(pkt, rssi);
    } else {
      // Non-blocking forwarding queue (Defect 1 fix)
      if (mesh.fwd_queue.count < FWD_QUEUE_SIZE) {
        mesh.fwd_queue.queue[mesh.fwd_queue.head] = pkt;
        mesh.fwd_queue.head = (mesh.fwd_queue.head + 1) % FWD_QUEUE_SIZE;
        mesh.fwd_queue.count++;
      } else {
        Serial.println("[ROUTE] Fwd queue full — dropping packet");
      }
    }
  }
}

// --- Handle BLE report ---
void mesh_handle_ble_report(const wsn_packet_t& pkt) {
  if (pkt.dst_id == MY_NODE_ID) {
    mac_send_ack(pkt);
    logPacket("RX", pkt);
    // Application handles it (N4 processes, others ignore)
    app_on_data_for_me(pkt, 0);
  }
}

// --- Handle CTRL command ---
void mesh_handle_ctrl(const wsn_packet_t& pkt) {
  ctrl_payload_t cp;
  getPayload(pkt, cp);
  logPacket("RX", pkt);

  switch (cp.cmd) {
    case CTRL_COLLISION_TEST:
      mesh.collision_test_trigger = true;
      Serial.println("[CTRL] Collision test triggered!");
      break;

    case CTRL_KILL_LINK:
      if (cp.param1 == MY_NODE_ID || cp.param2 == MY_NODE_ID) {
        uint8_t other = (cp.param1 == MY_NODE_ID) ? cp.param2 : cp.param1;
        mesh.killed_links[other] = true;
        mesh.neighbors[other].link_alive = false;
        Serial.printf("[CTRL] Software link kill: N%d↔N%d\n", cp.param1, cp.param2);
        routing_recompute_next_hop();
      }
      break;

    case CTRL_RESTORE_LINKS:
      for (int i = 1; i <= NUM_NODES; i++) {
        mesh.killed_links[i] = false;
      }
      Serial.println("[CTRL] All software link kills restored");
      routing_recompute_next_hop();
      break;
  }

  app_on_ctrl((ctrl_cmd_t)cp.cmd, cp.param1, cp.param2);
}

// =====================================================================
//                    MESH INIT & PERIODIC LOOP
// =====================================================================

// --- Initialize the full mesh stack ---
void mesh_init() {
  memset(&mesh, 0, sizeof(mesh));
  mesh.my_id = MY_NODE_ID;
  mesh.my_role = NODE_ROLES[MY_NODE_ID];
  mesh.current_cw = CW_MIN;
  mesh.next_hop_to_sink = 0;
  mesh.data_seq_counter = 1;
  mesh.ctrl_seq_counter = 1;
  mesh.fwd_queue.head = 0;
  mesh.fwd_queue.tail = 0;
  mesh.fwd_queue.count = 0;

  routing_init();
  tdma_init();

  // Init ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[MESH] ESP-NOW init FAILED!");
    return;
  }

  esp_now_register_recv_cb(mesh_on_recv);
  esp_now_register_send_cb(mesh_on_sent);

  // Add all other nodes as ESP-NOW peers
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

  // Add broadcast peer
  esp_now_peer_info_t bcast = {};
  memcpy(bcast.peer_addr, BROADCAST_MAC, 6);
  bcast.channel = 0;
  bcast.encrypt = false;
  esp_now_add_peer(&bcast);

  // Enable promiscuous mode for software CCA
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(promisc_rx_cb);

  Serial.printf("[MESH] Initialized: N%d (%s)\n", MY_NODE_ID, roleStr(mesh.my_role));
  Serial.printf("[MESH] MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    NODE_MAC[MY_NODE_ID][0], NODE_MAC[MY_NODE_ID][1],
    NODE_MAC[MY_NODE_ID][2], NODE_MAC[MY_NODE_ID][3],
    NODE_MAC[MY_NODE_ID][4], NODE_MAC[MY_NODE_ID][5]);
}

// --- Periodic mesh tasks — call from loop() ---
void mesh_loop() {
  uint32_t now = millis();

  // Process pending forwarding queue in main loop context (Defect 1 & 3 fix)
  // When synced, relays forward strictly during their assigned RELAY slot (or during CAP)
  tdma_epoch_t epoch = tdma_current_epoch();
  bool can_forward = (!mesh.tdma.synced) || tdma_is_my_relay_slot() || (epoch == EPOCH_CAP);
  if (can_forward) {
    while (mesh.fwd_queue.count > 0) {
      wsn_packet_t fwd = mesh.fwd_queue.queue[mesh.fwd_queue.tail];
      mesh.fwd_queue.tail = (mesh.fwd_queue.tail + 1) % FWD_QUEUE_SIZE;
      mesh.fwd_queue.count--;
      routing_forward(fwd);
    }
  }

  // Duty cycle / sleep tracking and modem sleep (Defect 5 fix)
  if (epoch != mesh.tdma.last_epoch) {
    uint32_t dt = now - mesh.tdma.last_epoch_switch_ms;
    if (mesh.tdma.last_epoch == EPOCH_SLEEP) {
      mesh.tdma.sleep_time_ms += dt;
    } else {
      mesh.tdma.active_time_ms += dt;
    }
    // Put Wi-Fi modem to sleep during sleep epoch (N4 stays awake for WiFi/cloud)
    if (MY_NODE_ID != 4) {
      if (epoch == EPOCH_SLEEP) {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
      } else {
        esp_wifi_set_ps(WIFI_PS_NONE);
      }
    }
    mesh.tdma.last_epoch = epoch;
    mesh.tdma.last_epoch_switch_ms = now;
  }

  // 0. Channel scan for nodes 1, 2, 3: hop channels every 1.2s until synced with N4
  if (MY_NODE_ID != 4 && !mesh.tdma.synced) {
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

  // 1. Periodic HELLO broadcast (every 2s during CAP or anytime if not synced)
  if (now - mesh.last_hello_ms >= HELLO_INTERVAL_MS) {
    if (!mesh.tdma.synced || epoch == EPOCH_CAP) {
      routing_send_hello();
      mesh.last_hello_ms = now;
    }
  }

  // 2. HELLO timeout check (mark dead neighbors)
  routing_check_timeouts();

  // 3. N4: Send beacon at start of each superframe
  if (MY_NODE_ID == 4 && mesh.tdma.synced) {
    static uint32_t last_beacon = 0;
    if (now - last_beacon >= SUPERFRAME_MS) {
      tdma_send_beacon();
      last_beacon = now;
    }
  } else if (MY_NODE_ID == 4 && !mesh.tdma.synced) {
    mesh.tdma.synced = true;
    mesh.tdma.clock_offset = 0;
    tdma_send_beacon();
  }

  // 4. N3: Periodic schedule rebroadcast (every 6 seconds so nodes never stay unscheduled)
  if (MY_NODE_ID == 3 && mesh.tdma.synced) {
    static uint32_t last_schedule_rebroadcast = 0;
    if (now - last_schedule_rebroadcast >= 6000) {
      tdma_send_default_schedule("PERIODIC: schedule refresh");
      last_schedule_rebroadcast = now;
    }
  }
}

// =====================================================================
//            CONVENIENCE: Send sensor data to sink via mesh
// =====================================================================

// Build and send a MSG_DATA packet through the routing layer
bool mesh_send_sensor_data(float v1, float v2, float v3, const char* sensor_type) {
  if (mesh.next_hop_to_sink == 0) {
    Serial.println("[MESH] Cannot send data — no route to sink");
    return false;
  }

  wsn_packet_t pkt;
  initPacket(pkt, MSG_DATA, mesh.next_hop_to_sink, MY_NODE_ID, 4, mesh.data_seq_counter++);
  pkt.route_path[0] = MY_NODE_ID;

  sensor_payload_t sp;
  sp.val1 = v1;
  sp.val2 = v2;
  sp.val3 = v3;
  strncpy(sp.sensor_type, sensor_type, sizeof(sp.sensor_type) - 1);
  sp.sensor_type[sizeof(sp.sensor_type) - 1] = '\0';
  setPayload(pkt, sp);

  return mac_send(pkt);
}

// Build and send a MSG_AGG packet through the routing layer (N3)
bool mesh_send_agg_data(const agg_payload_t& agg) {
  if (mesh.next_hop_to_sink == 0) {
    Serial.println("[MESH] Cannot send AGG — no route to sink");
    return false;
  }

  wsn_packet_t pkt;
  initPacket(pkt, MSG_AGG, mesh.next_hop_to_sink, MY_NODE_ID, 4, mesh.data_seq_counter++);
  pkt.route_path[0] = MY_NODE_ID;
  setPayload(pkt, agg);

  return mac_send(pkt);
}

// Send BLE report to sink
bool mesh_send_ble_report(const ble_report_payload_t& rpt) {
  if (MY_NODE_ID == 4) {
    return true;  // N4 processes locally, no transmission needed (Defect fix: avoid null dereference)
  }
  if (mesh.next_hop_to_sink == 0) {
    Serial.println("[MESH] Cannot send BLE report — no route to sink");
    return false;
  }

  wsn_packet_t pkt;
  initPacket(pkt, MSG_BLE_REPORT, mesh.next_hop_to_sink, MY_NODE_ID, 4, mesh.ctrl_seq_counter++);
  setPayload(pkt, rpt);
  return mac_send(pkt);
}

// =====================================================================
//           CONVENIENCE: Collision test sender (N1 and N2)
// =====================================================================

// Called from loop() when collision_test_trigger is set
void mesh_collision_test_send() {
  if (!mesh.collision_test_trigger) return;
  mesh.collision_test_trigger = false;

  Serial.println("[COLLISION TEST] Sending data immediately (no TDMA slot check)!");

  // Build a test packet
  wsn_packet_t pkt;
  initPacket(pkt, MSG_DATA, mesh.next_hop_to_sink, MY_NODE_ID, 4, mesh.ctrl_seq_counter++);
  sensor_payload_t sp;
  sp.val1 = 999.0f;
  sp.val2 = 999.0f;
  sp.val3 = 999.0f;
  strncpy(sp.sensor_type, "colltest", sizeof(sp.sensor_type) - 1);
  setPayload(pkt, sp);

  // Send immediately — CCA will detect contention
  mac_send(pkt);
}

// Print mesh status summary
void mesh_print_status() {
  Serial.println("\n====== MESH STATUS ======");
  Serial.printf("Node: N%d (%s)\n", MY_NODE_ID, roleStr(mesh.my_role));
  Serial.printf("Next hop to sink: N%d\n", mesh.next_hop_to_sink);
  Serial.printf("TDMA synced: %s | Frame: %lu\n",
    mesh.tdma.synced ? "YES" : "NO", mesh.tdma.frame_number);

  uint32_t total_time = mesh.tdma.active_time_ms + mesh.tdma.sleep_time_ms;
  float duty_cycle = (total_time > 0) ? (mesh.tdma.active_time_ms * 100.0f / total_time) : 100.0f;
  Serial.printf("Duty cycle: %.1f%% (active=%lu ms, sleep=%lu ms)\n",
    duty_cycle, mesh.tdma.active_time_ms, mesh.tdma.sleep_time_ms);

  Serial.println("--- Neighbors ---");
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    route_neighbor_t& nb = mesh.neighbors[i];
    Serial.printf("  N%d: %s | RSSI=%d | ACK=%.0f%% | hops=%d | kill=%s | retries=%u\n",
      i, nb.link_alive ? "ALIVE" : "DEAD",
      nb.wifi_rssi, nb.ack_success_rate * 100,
      nb.hops_to_sink, mesh.killed_links[i] ? "YES" : "NO",
      nb.reported_retries);
  }
  Serial.printf("--- MAC Stats ---\n");
  Serial.printf("  CCA busy=%lu backoffs=%lu retries=%lu drops=%lu ACKs=%lu timeouts=%lu\n",
    mesh.mac_stats.cca_busy_count, mesh.mac_stats.total_backoffs,
    mesh.mac_stats.total_retries, mesh.mac_stats.total_drops,
    mesh.mac_stats.total_acks_ok, mesh.mac_stats.total_acks_timeout);
  if (MY_NODE_ID == 4) {
    Serial.printf("  Sent=%lu Fwd=%lu RxAtSink=%lu\n",
      mesh.pkts_sent, mesh.pkts_forwarded, mesh.pkts_received_at_sink);
  } else {
    Serial.printf("  Sent=%lu Fwd=%lu\n",
      mesh.pkts_sent, mesh.pkts_forwarded);
  }
  Serial.println("=========================\n");
}

#endif // WSN_MESH_H
