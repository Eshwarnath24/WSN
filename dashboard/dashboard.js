import { db } from "./firebase-config.js";
import {
  ref,
  set,
  onValue,
  query,
  limitToLast,
  orderByKey
} from "https://www.gstatic.com/firebasejs/10.12.0/firebase-database.js";
import { createNodeCard } from "./node-card.js";

const NODE_IDS = ["N1", "N2", "N3"];
const container = document.getElementById("grid");
const cards = {};
let totalPackets = 0;

// Initialize Cards
NODE_IDS.forEach((nodeId) => {
  cards[nodeId] = createNodeCard(container, nodeId);
});
updateActiveCount();

// Monitor Firebase Realtime Database connection status
onValue(ref(db, ".info/connected"), (snapshot) => {
  if (snapshot.val() === true) {
    console.log("[Firebase RTDB] Connected successfully to Realtime Database.");
  } else {
    console.warn("[Firebase RTDB] Disconnected / Waiting for connection...");
  }
});

// ---------------------------------------------------------------
// COMBINED QoS AGGREGATION
// ---------------------------------------------------------------
const perNodeQoS = {};

function updateCombinedQoS() {
  const ids = NODE_IDS;
  let totalExpected = 0, totalReceived = 0, totalLost = 0;
  let totalDup = 0, totalOOO = 0;
  let rssiSum = 0, rssiCount = 0;
  let ackSum = 0, ackCount = 0;
  const perNode = {};

  ids.forEach((id) => {
    const q = perNodeQoS[id];
    if (!q) {
      perNode[id] = { pdr: '—', expected: '—', received: '—', lost: '—', dup: '—', ooo: '—', rssi: '—', ack: '—' };
      return;
    }

    const exp = Number(q.expected ?? 0);
    const rec = Number(q.received ?? 0);
    const lost = Number(q.lost ?? 0);
    const dup = Number(q.duplicates ?? 0);
    const ooo = Number(q.out_of_order ?? 0);
    const pdr = Number(q.pdr ?? 0);
    const rssi = q.rssi_dbm !== undefined ? Number(q.rssi_dbm) : null;
    const ack = q.last_ack_delay_ms !== undefined ? Number(q.last_ack_delay_ms) : null;

    totalExpected += exp;
    totalReceived += rec;
    totalLost += lost;
    totalDup += dup;
    totalOOO += ooo;

    if (rssi !== null) { rssiSum += rssi; rssiCount++; }
    if (ack !== null) { ackSum += ack; ackCount++; }

    perNode[id] = {
      pdr: pdr.toFixed(1) + '%',
      expected: exp,
      received: rec,
      lost: lost,
      dup: dup,
      ooo: ooo,
      rssi: rssi !== null ? rssi + ' dBm' : '—',
      ack: ack !== null ? ack + ' ms' : '—'
    };
  });

  // Overall PDR
  const overallPDR = totalExpected > 0 ? (totalReceived * 100.0 / totalExpected) : 0;
  const avgRSSI = rssiCount > 0 ? (rssiSum / rssiCount) : null;
  const avgAck = ackCount > 0 ? (ackSum / ackCount) : null;

  // Update DOM
  const setVal = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };

  // PDR card — color based on value
  const pdrEl = document.getElementById('nq-pdr-val');
  if (pdrEl) {
    pdrEl.textContent = totalExpected > 0 ? overallPDR.toFixed(1) : '—';
    pdrEl.className = 'nq-card-value ' + (overallPDR >= 95 ? 'green' : overallPDR >= 80 ? 'amber' : 'red');
  }
  // PDR card border glow
  const pdrCard = document.getElementById('nq-pdr-card');
  if (pdrCard) {
    pdrCard.className = 'nq-card nq-card-highlight' + (overallPDR >= 95 ? ' nq-glow-green' : overallPDR >= 80 ? ' nq-glow-amber' : ' nq-glow-red');
  }

  setVal('nq-expected-val', totalExpected || '—');
  setVal('nq-received-val', totalReceived || '—');

  // Lost — color
  const lostEl = document.getElementById('nq-lost-val');
  if (lostEl) {
    lostEl.textContent = totalLost;
    lostEl.className = 'nq-card-value ' + (totalLost === 0 ? 'green' : totalLost <= 10 ? 'amber' : 'red');
  }

  setVal('nq-dup-val', totalDup);
  setVal('nq-ooo-val', totalOOO);
  setVal('nq-rssi-val', avgRSSI !== null ? avgRSSI.toFixed(0) : '—');

  // RSSI color (closer to 0 is better)
  const rssiValEl = document.getElementById('nq-rssi-val');
  if (rssiValEl && avgRSSI !== null) {
    rssiValEl.className = 'nq-card-value ' + (avgRSSI > -50 ? 'green' : avgRSSI > -70 ? 'cyan' : avgRSSI > -85 ? 'amber' : 'red');
  }

  setVal('nq-ack-val', avgAck !== null ? avgAck.toFixed(0) : '—');

  // Per-node breakdown chips
  const updateBreakdown = (elId, field) => {
    const el = document.getElementById(elId);
    if (!el) return;
    el.innerHTML = ids.map((id) => {
      const val = perNode[id] ? perNode[id][field] : '—';
      return `<span class="nq-chip">${id}: ${val}</span>`;
    }).join('');
  };

  updateBreakdown('nq-pdr-breakdown', 'pdr');
  updateBreakdown('nq-expected-breakdown', 'expected');
  updateBreakdown('nq-received-breakdown', 'received');
  updateBreakdown('nq-lost-breakdown', 'lost');
  updateBreakdown('nq-dup-breakdown', 'dup');
  updateBreakdown('nq-ooo-breakdown', 'ooo');
  updateBreakdown('nq-rssi-breakdown', 'rssi');
  updateBreakdown('nq-ack-breakdown', 'ack');
}

// ---------------------------------------------------------------
// FIREBASE LISTENERS
// ---------------------------------------------------------------

// Track previous snapshot keys per node for diff-based updates
const prevSnapshotKeys = {};

NODE_IDS.forEach((nodeId) => {
  // ------ Status listener (Firebase is single source of truth) ------
  onValue(
    ref(db, `nodes/${nodeId}/status`),
    (snapshot) => {
      const status = snapshot.val();
      const isActive = status === "active" || status === true;
      cards[nodeId].setStatus(isActive);
      updateActiveCount();
    },
    (err) => {
      console.warn(`[Firebase] Status listener error for ${nodeId}:`, err);
    }
  );

  // ------ Sensor type listener ------
  onValue(
    ref(db, `nodes/${nodeId}/sensor_type`),
    (snapshot) => {
      const type = snapshot.val();
      if (type) {
        cards[nodeId].setSensorType(type);
      }
    },
    (err) => {
      console.warn(`[Firebase] Sensor type listener error for ${nodeId}:`, err);
    }
  );

  // ------ QoS listener — feed into combined aggregator ------
  onValue(
    ref(db, `nodes/${nodeId}/qos`),
    (snapshot) => {
      const qos = snapshot.val();
      if (qos) {
        perNodeQoS[nodeId] = qos;
        updateCombinedQoS();
      }
    },
    (err) => {
      console.warn(`[Firebase] QoS listener error for ${nodeId}:`, err);
    }
  );

  // ------ Data listener (single onValue with diff) ------
  // Uses a single onValue on limitToLast(30). On each fire we diff
  // against the previous set of keys to find only the NEW entries,
  // then push only those to the chart. This avoids onChildAdded's
  // re-fire-on-existing-children race and its sliding-window latency.
  const dataQuery = query(ref(db, `nodes/${nodeId}/data`), orderByKey(), limitToLast(30));
  prevSnapshotKeys[nodeId] = new Set();
  let isFirstLoad = true;

  onValue(
    dataQuery,
    (snapshot) => {
      if (!snapshot.exists()) return;

      const entries = [];
      const currentKeys = new Set();

      snapshot.forEach((child) => {
        currentKeys.add(child.key);
        entries.push({ key: child.key, ...child.val() });
      });

      if (isFirstLoad) {
        // First load: populate chart with all historical points
        entries.forEach((entry) => {
          const v1 = Number(entry.val1 ?? 0);
          const v2 = Number(entry.val2 ?? entry.val1 ?? 0);
          const v3 = Number(entry.val3 ?? entry.val1 ?? 0);
          const label = formatLabel(entry);
          cards[nodeId].pushChartPoint(label, v1, v2, v3);
          cards[nodeId].setLatestValues(entry.seq_num, v1, v2, v3);
        });
        isFirstLoad = false;
      } else {
        // Subsequent fires: diff to find only new entries
        const newEntries = entries.filter((e) => !prevSnapshotKeys[nodeId].has(e.key));

        newEntries.forEach((entry) => {
          totalPackets++;
          document.getElementById("total-packets").textContent = totalPackets.toLocaleString();

          const v1 = Number(entry.val1 ?? 0);
          const v2 = Number(entry.val2 ?? entry.val1 ?? 0);
          const v3 = Number(entry.val3 ?? entry.val1 ?? 0);
          const label = formatLabel(entry);

          cards[nodeId].pushChartPoint(label, v1, v2, v3);
          cards[nodeId].setLatestValues(entry.seq_num, v1, v2, v3);
        });
      }

      // Update previous keys for next diff
      prevSnapshotKeys[nodeId] = currentKeys;
    },
    (err) => {
      console.warn(`[Firebase] Data listener error for ${nodeId}:`, err);
    }
  );
});

// ---------------------------------------------------------------
// HELPERS
// ---------------------------------------------------------------

/** Format a readable time label from a data entry */
function formatLabel(entry) {
  // Use the timestamp from the entry if available, otherwise use seq_num
  if (entry.timestamp && entry.timestamp > 1000000) {
    // Looks like a unix-ish timestamp (probably millis since boot)
    // Use current wall-clock since ESP timestamps are millis-since-boot
    return new Date().toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  }
  return String(entry.seq_num ?? '');
}

function updateActiveCount() {
  let activeCount = 0;
  NODE_IDS.forEach((id) => {
    if (cards[id] && cards[id].isActive()) {
      activeCount++;
    }
  });
  const countEl = document.getElementById("active-count");
  if (countEl) countEl.textContent = `${activeCount}/${NODE_IDS.length}`;
}

// ---------------------------------------------------------------
// 1. DEMO EVALUATION CONTROLS (Dispatches to /control in Firebase)
// ---------------------------------------------------------------
function sendControlCommand(cmd, p1 = 0, p2 = 0, description = "") {
  const statusEl = document.getElementById("ctrl-last-action");
  if (statusEl) {
    statusEl.textContent = `Dispatching: ${description || cmd}...`;
    statusEl.style.color = "#facc15";
  }
  set(ref(db, "control"), {
    cmd: cmd,
    p1: p1,
    p2: p2,
    timestamp: Date.now()
  })
    .then(() => {
      if (statusEl) {
        statusEl.textContent = `Active: ${description || cmd}`;
        statusEl.style.color = "#4ade80";
      }
    })
    .catch((err) => {
      console.error("[Control] Failed to send command:", err);
      if (statusEl) {
        statusEl.textContent = `Error: ${err.message}`;
        statusEl.style.color = "#f87171";
      }
    });
}

document.getElementById("btn-collision")?.addEventListener("click", () => {
  sendControlCommand("collision", 0, 0, "CSMA/CA Contention Test");
});

document.getElementById("btn-kill")?.addEventListener("click", () => {
  sendControlCommand("kill", 3, 4, "Link N3↔N4 Killed (Failover)");
});

document.getElementById("btn-restore")?.addEventListener("click", () => {
  sendControlCommand("restore", 0, 0, "Links Restored (Primary Multi-Hop)");
});

document.getElementById("btn-query")?.addEventListener("click", () => {
  sendControlCommand("status", 0, 0, "Network Status Probed");
});

// ---------------------------------------------------------------
// 2. EXTENDED PERFORMANCE METRICS (/perf/throughput, /perf/mac, /perf/duty_cycle)
// ---------------------------------------------------------------
onValue(ref(db, "perf/throughput"), (snap) => {
  const val = snap.val();
  const el = document.getElementById("nq-throughput-val");
  if (el && val && val.kbps !== undefined) {
    el.textContent = Number(val.kbps).toFixed(2);
  }
});

onValue(ref(db, "perf/mac"), (snap) => {
  const val = snap.val();
  const el = document.getElementById("nq-retries-val");
  if (el && val && val.network_retries !== undefined) {
    el.textContent = val.network_retries;
  }
});

onValue(ref(db, "perf/duty_cycle"), (snap) => {
  const val = snap.val();
  const el = document.getElementById("nq-duty-val");
  if (el && val && val.duty_cycle_pct !== undefined) {
    el.textContent = Number(val.duty_cycle_pct).toFixed(1);
  }
});

// ---------------------------------------------------------------
// 3. MESH TOPOLOGY & ROUTING STATUS (/route/topology)
// ---------------------------------------------------------------
onValue(ref(db, "route/topology"), (snap) => {
  const topo = snap.val();
  if (!topo) return;

  const n1Next = document.getElementById("n1-next-hop");
  const n2Next = document.getElementById("n2-next-hop");
  const n3Next = document.getElementById("n3-next-hop");
  if (n1Next) n1Next.textContent = topo.N1_next || "—";
  if (n2Next) n2Next.textContent = topo.N2_next || "—";
  if (n3Next) n3Next.textContent = topo.N3_next || "—";

  const n1Badge = document.getElementById("n1-link-badge");
  const n2Badge = document.getElementById("n2-link-badge");
  const n3Badge = document.getElementById("n3-link-badge");
  if (n1Badge) {
    n1Badge.textContent = topo.N1_alive ? "ALIVE" : "DEAD";
    n1Badge.className = "topo-badge " + (topo.N1_alive ? "badge-alive" : "badge-dead");
  }
  if (n2Badge) {
    n2Badge.textContent = topo.N2_alive ? "ALIVE" : "DEAD";
    n2Badge.className = "topo-badge " + (topo.N2_alive ? "badge-alive" : "badge-dead");
  }
  if (n3Badge) {
    n3Badge.textContent = topo.N3_alive ? "ALIVE" : "DEAD";
    n3Badge.className = "topo-badge " + (topo.N3_alive ? "badge-alive" : "badge-dead");
  }

  // Update SVG Topology Graph Lines
  const l34 = document.getElementById("link-n3-n4");
  const l13 = document.getElementById("link-n1-n3");
  const l23 = document.getElementById("link-n2-n3");
  const l14 = document.getElementById("link-n1-n4");
  const l24 = document.getElementById("link-n2-n4");

  if (l34) {
    l34.setAttribute("stroke", topo.N3_alive ? "#00d2ff" : "#ef4444");
    l34.setAttribute("stroke-dasharray", topo.N3_alive ? "none" : "4,4");
  }
  if (topo.N1_next === "N4") {
    l14?.setAttribute("stroke", "#facc15");
    l14?.setAttribute("stroke-width", "2");
    l13?.setAttribute("stroke", "rgba(255,255,255,0.08)");
  } else {
    l13?.setAttribute("stroke", "#00d2ff");
    l14?.setAttribute("stroke", "rgba(255,255,255,0.08)");
  }
  if (topo.N2_next === "N4") {
    l24?.setAttribute("stroke", "#facc15");
    l24?.setAttribute("stroke-width", "2");
    l23?.setAttribute("stroke", "rgba(255,255,255,0.08)");
  } else {
    l23?.setAttribute("stroke", "#00d2ff");
    l24?.setAttribute("stroke", "rgba(255,255,255,0.08)");
  }

  const statusChip = document.getElementById("topo-status-chip");
  if (statusChip) {
    statusChip.textContent = (!topo.N3_alive) ? "Failover Mode (Direct N4)" : "Clustered Multi-Hop";
    statusChip.style.color = (!topo.N3_alive) ? "#facc15" : "#00d2ff";
  }
});

// ---------------------------------------------------------------
// 4. CLUSTER HEAD DATA AGGREGATION (/agg/data)
// ---------------------------------------------------------------
const aggQuery = query(ref(db, "agg/data"), orderByKey(), limitToLast(1));
onValue(aggQuery, (snap) => {
  if (!snap.exists()) return;
  snap.forEach((child) => {
    const d = child.val();
    const redVal = document.getElementById("agg-reduction-val");
    const meter = document.getElementById("agg-meter-fill");
    if (redVal) redVal.textContent = (d.reduction_pct ? Number(d.reduction_pct).toFixed(1) : "0.0") + "%";
    if (meter) meter.style.width = Math.min(100, Math.max(0, d.reduction_pct || 0)) + "%";

    const bytesIn = document.getElementById("agg-bytes-in");
    const bytesOut = document.getElementById("agg-bytes-out");
    if (bytesIn) bytesIn.textContent = (d.bytes_in || 0) + " B";
    if (bytesOut) bytesOut.textContent = (d.bytes_out || 0) + " B";

    const n1Sum = document.getElementById("agg-n1-summary");
    if (n1Sum && d.n1_mean_z !== undefined) {
      n1Sum.textContent = `Z: ${Number(d.n1_mean_z).toFixed(2)} | Pk: ${Number(d.n1_max || 0).toFixed(1)}`;
    }
    const n2Sum = document.getElementById("agg-n2-summary");
    if (n2Sum && d.n2_flame_max !== undefined) {
      n2Sum.textContent = `${Number(d.n2_flame_max).toFixed(0)} (${d.n2_flame_digital === 0 ? "FIRE" : "OK"})`;
    }

    const irBadge = document.getElementById("agg-ir-badge");
    if (irBadge) {
      const hasObstacle = d.n3_ir === 1;
      irBadge.textContent = hasObstacle ? "OBSTACLE DETECTED" : "CLEAR";
      irBadge.className = "topo-badge " + (hasObstacle ? "badge-dead" : "badge-alive");
    }

    const hopsChip = document.getElementById("agg-hops-chip");
    if (hopsChip) {
      hopsChip.textContent = `${d.hops || 1} Hop${(d.hops > 1 ? "s" : "")} to Sink`;
    }
  });
});

// ---------------------------------------------------------------
// 5. BLE 2D LOCALIZATION TRILATERATION ENGINE (/ble/N*)
// ---------------------------------------------------------------
const bleNodeData = {
  N1: { trueX: 0, trueY: 0, estX: null, estY: null, error: null },
  N2: { trueX: 2, trueY: 0, estX: null, estY: null, error: null },
  N3: { trueX: 1, trueY: 2, estX: null, estY: null, error: null },
  N4: { trueX: 1, trueY: 1, estX: 1, estY: 1, error: 0 }
};

function renderBleMap() {
  const canvas = document.getElementById("ble-map-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  // Set physical display resolution
  const w = canvas.clientWidth || 300;
  const h = canvas.clientHeight || 200;
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }

  ctx.clearRect(0, 0, w, h);

  // Coordinate space: (X: -0.5 to 2.5m, Y: -0.5 to 2.5m)
  const pad = 24;
  const toScreenX = (x) => pad + (x / 2.5) * (w - 2 * pad);
  const toScreenY = (y) => (h - pad) - (y / 2.5) * (h - 2 * pad);

  // Draw Grid lines
  ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
  ctx.lineWidth = 1;
  for (let grid = 0; grid <= 2.5; grid += 0.5) {
    const sx = toScreenX(grid);
    const sy = toScreenY(grid);
    ctx.beginPath();
    ctx.moveTo(sx, pad);
    ctx.lineTo(sx, h - pad);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(pad, sy);
    ctx.lineTo(w - pad, sy);
    ctx.stroke();
  }

  // Draw each node's True Anchor and Estimated Location
  Object.keys(bleNodeData).forEach((nid) => {
    const data = bleNodeData[nid];
    const tx = toScreenX(data.trueX);
    const ty = toScreenY(data.trueY);

    // True position: Solid diamond
    ctx.fillStyle = (nid === "N4") ? "#22c55e" : (nid === "N3") ? "#c084fc" : "#00d2ff";
    ctx.beginPath();
    ctx.moveTo(tx, ty - 6);
    ctx.lineTo(tx + 6, ty);
    ctx.lineTo(tx, ty + 6);
    ctx.lineTo(tx - 6, ty);
    ctx.closePath();
    ctx.fill();

    // Node label
    ctx.fillStyle = "#94a3b8";
    ctx.font = "9px 'JetBrains Mono', monospace";
    ctx.fillText(`${nid}`, tx + 8, ty - 2);

    // Estimated position (if available)
    if (data.estX !== null && data.estY !== null) {
      const ex = toScreenX(data.estX);
      const ey = toScreenY(data.estY);

      // Error vector line
      ctx.strokeStyle = "rgba(250, 204, 21, 0.4)";
      ctx.setLineDash([2, 2]);
      ctx.beginPath();
      ctx.moveTo(tx, ty);
      ctx.lineTo(ex, ey);
      ctx.stroke();
      ctx.setLineDash([]);

      // Estimated point: Glowing circle
      ctx.fillStyle = "#facc15";
      ctx.beginPath();
      ctx.arc(ex, ey, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  });
}

// Subscribe to BLE telemetry for each node
["N1", "N2", "N3"].forEach((nid) => {
  onValue(ref(db, `ble/${nid}`), (snap) => {
    const data = snap.val();
    if (!data) return;

    if (bleNodeData[nid]) {
      bleNodeData[nid].estX = Number(data.est_x ?? 0);
      bleNodeData[nid].estY = Number(data.est_y ?? 0);
      bleNodeData[nid].error = Number(data.error_m ?? 0);
    }

    const row = document.getElementById(`ble-${nid.toLowerCase()}-pos`);
    if (row) {
      row.textContent = `est=(${Number(data.est_x || 0).toFixed(2)}, ${Number(data.est_y || 0).toFixed(2)}) err=${Number(data.error_m || 0).toFixed(2)}m`;
    }

    renderBleMap();
  });
});

// Periodic redraw for responsiveness
window.addEventListener("resize", renderBleMap);
setTimeout(renderBleMap, 1000);