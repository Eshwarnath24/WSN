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

const setText = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
const num = (v, d = 1) => (v === undefined || v === null || Number.isNaN(Number(v))) ? "—" : Number(v).toFixed(d);

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
// GATEWAY LIVENESS: N4 pushes metrics every 5 s. If nothing arrives for
// 15 s the data on screen is stale (gateway offline or no Wi-Fi).
// ---------------------------------------------------------------
let lastGatewayUpdate = 0;
function markGatewayUpdate() { lastGatewayUpdate = Date.now(); refreshGatewayStatus(); }
function refreshGatewayStatus() {
  const el = document.getElementById("gateway-status");
  if (!el) return;
  if (!lastGatewayUpdate) { el.textContent = "waiting…"; el.style.color = "#94a3b8"; return; }
  const age = (Date.now() - lastGatewayUpdate) / 1000;
  if (age > 15) {
    el.textContent = `STALE (${age.toFixed(0)} s)`;
    el.style.color = "#f87171";
  } else {
    el.textContent = "ONLINE";
    el.style.color = "#4ade80";
  }
}
setInterval(refreshGatewayStatus, 2000);

// ---------------------------------------------------------------
// COMBINED QoS AGGREGATION (per-node values computed by N4 on unique packets)
// ---------------------------------------------------------------
const perNodeQoS = {};

function updateCombinedQoS() {
  const ids = NODE_IDS;
  let totalExpected = 0, totalReceived = 0, totalLost = 0;
  let totalDup = 0, totalOOO = 0;
  let rssiSum = 0, rssiCount = 0;
  let delaySum = 0, delayCount = 0;
  const perNode = {};

  ids.forEach((id) => {
    const q = perNodeQoS[id];
    if (!q) {
      perNode[id] = { pdr: '—', expected: '—', received: '—', lost: '—', dup: '—', ooo: '—', rssi: '—', delay: '—' };
      return;
    }

    const exp = Number(q.expected ?? 0);
    const rec = Number(q.received ?? 0);
    const lost = Number(q.lost ?? 0);
    const dup = Number(q.duplicates ?? 0);
    const ooo = Number(q.out_of_order ?? 0);
    const pdr = Number(q.pdr ?? 0);
    const rssi = (q.rssi_dbm !== undefined && Number(q.rssi_dbm) !== 0) ? Number(q.rssi_dbm) : null;
    const delay = q.avg_delay_ms !== undefined ? Number(q.avg_delay_ms) : null;

    totalExpected += exp;
    totalReceived += rec;
    totalLost += lost;
    totalDup += dup;
    totalOOO += ooo;

    if (rssi !== null) { rssiSum += rssi; rssiCount++; }
    if (delay !== null && rec > 0) { delaySum += delay; delayCount++; }

    perNode[id] = {
      pdr: pdr.toFixed(1) + '%',
      expected: exp,
      received: rec,
      lost: `${lost} (${num(q.loss_pct)}%)`,
      dup: dup,
      ooo: ooo,
      rssi: rssi !== null ? rssi + ' dBm' : '—',
      delay: delay !== null ? delay.toFixed(0) + ' ms' : '—'
    };
  });

  const overallPDR = totalExpected > 0 ? (totalReceived * 100.0 / totalExpected) : 0;
  const avgRSSI = rssiCount > 0 ? (rssiSum / rssiCount) : null;
  const avgDelay = delayCount > 0 ? (delaySum / delayCount) : null;

  const pdrEl = document.getElementById('nq-pdr-val');
  if (pdrEl) {
    pdrEl.textContent = totalExpected > 0 ? overallPDR.toFixed(1) : '—';
    pdrEl.className = 'nq-card-value ' + (overallPDR >= 95 ? 'green' : overallPDR >= 80 ? 'amber' : 'red');
  }
  const pdrCard = document.getElementById('nq-pdr-card');
  if (pdrCard) {
    pdrCard.className = 'nq-card nq-card-highlight' + (overallPDR >= 95 ? ' nq-glow-green' : overallPDR >= 80 ? ' nq-glow-amber' : ' nq-glow-red');
  }

  setText('nq-expected-val', totalExpected || '—');
  setText('nq-received-val', totalReceived || '—');

  const lostEl = document.getElementById('nq-lost-val');
  if (lostEl) {
    lostEl.textContent = totalLost;
    lostEl.className = 'nq-card-value ' + (totalLost === 0 ? 'green' : totalLost <= 10 ? 'amber' : 'red');
  }

  setText('nq-dup-val', totalDup);
  setText('nq-ooo-val', totalOOO);
  setText('nq-rssi-val', avgRSSI !== null ? avgRSSI.toFixed(0) : '—');

  const rssiValEl = document.getElementById('nq-rssi-val');
  if (rssiValEl && avgRSSI !== null) {
    rssiValEl.className = 'nq-card-value ' + (avgRSSI > -50 ? 'green' : avgRSSI > -70 ? 'cyan' : avgRSSI > -85 ? 'amber' : 'red');
  }

  setText('nq-ack-val', avgDelay !== null ? avgDelay.toFixed(0) : '—');

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
  updateBreakdown('nq-ack-breakdown', 'delay');
}

// ---------------------------------------------------------------
// FIREBASE LISTENERS (per node)
// ---------------------------------------------------------------
const prevSnapshotKeys = {};

NODE_IDS.forEach((nodeId) => {
  // Status (written by N4's liveness watchdog: active / inactive)
  onValue(
    ref(db, `nodes/${nodeId}/status`),
    (snapshot) => {
      const status = snapshot.val();
      const isActive = status === "active" || status === true;
      cards[nodeId].setStatus(isActive);
      updateActiveCount();
    },
    (err) => console.warn(`[Firebase] Status listener error for ${nodeId}:`, err)
  );

  onValue(
    ref(db, `nodes/${nodeId}/sensor_type`),
    (snapshot) => {
      const type = snapshot.val();
      if (type) cards[nodeId].setSensorType(type);
    },
    (err) => console.warn(`[Firebase] Sensor type listener error for ${nodeId}:`, err)
  );

  onValue(
    ref(db, `nodes/${nodeId}/qos`),
    (snapshot) => {
      const qos = snapshot.val();
      if (qos) {
        perNodeQoS[nodeId] = qos;
        updateCombinedQoS();
      }
    },
    (err) => console.warn(`[Firebase] QoS listener error for ${nodeId}:`, err)
  );

  // Sensor data (single onValue on the last 30 entries, diffed by key)
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

      const toPush = isFirstLoad ? entries : entries.filter((e) => !prevSnapshotKeys[nodeId].has(e.key));
      toPush.forEach((entry) => {
        if (!isFirstLoad) {
          totalPackets++;
          setText("total-packets", totalPackets.toLocaleString());
        }
        const v1 = Number(entry.val1 ?? 0);
        const v2 = Number(entry.val2 ?? entry.val1 ?? 0);
        const v3 = Number(entry.val3 ?? entry.val1 ?? 0);
        cards[nodeId].pushChartPoint(formatLabel(entry), v1, v2, v3);
        cards[nodeId].setLatestValues(entry.seq_num, v1, v2, v3);
      });
      isFirstLoad = false;
      prevSnapshotKeys[nodeId] = currentKeys;
    },
    (err) => console.warn(`[Firebase] Data listener error for ${nodeId}:`, err)
  );
});

// ---------------------------------------------------------------
// HELPERS
// ---------------------------------------------------------------
function formatLabel(entry) {
  // ESP timestamps are millis-since-boot, so label with the browser wall clock
  if (entry.timestamp && entry.timestamp > 1000) {
    return new Date().toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  }
  return String(entry.seq_num ?? '');
}

function updateActiveCount() {
  let activeCount = 0;
  NODE_IDS.forEach((id) => {
    if (cards[id] && cards[id].isActive()) activeCount++;
  });
  setText("active-count", `${activeCount}/${NODE_IDS.length}`);
}

// ---------------------------------------------------------------
// 1. DEMO EVALUATION CONTROLS (written to /control, polled by N4)
// ---------------------------------------------------------------
function sendControlCommand(cmd, p1 = 0, p2 = 0, description = "") {
  const statusEl = document.getElementById("ctrl-last-action");
  if (statusEl) {
    statusEl.textContent = `Dispatching: ${description || cmd}...`;
    statusEl.style.color = "#facc15";
  }
  set(ref(db, "control"), { cmd, p1, p2, timestamp: Date.now() })
    .then(() => {
      if (statusEl) {
        statusEl.textContent = `Sent to N4 (applied within ~3 s): ${description || cmd}`;
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
  sendControlCommand("collision", 0, 0, "CSMA/CA Contention Test (N1+N2 at next CAP)");
});
document.getElementById("btn-kill")?.addEventListener("click", () => {
  sendControlCommand("kill", 3, 4, "Link N3↔N4 Killed (software failure injection)");
});
document.getElementById("btn-restore")?.addEventListener("click", () => {
  sendControlCommand("restore", 0, 0, "Links Restored");
});
document.getElementById("btn-query")?.addEventListener("click", () => {
  sendControlCommand("status", 0, 0, "Status printed on N4 serial + metrics pushed");
});

// ---------------------------------------------------------------
// 2. PERFORMANCE METRICS (/perf/*)
// ---------------------------------------------------------------
onValue(ref(db, "perf/throughput"), (snap) => {
  const val = snap.val();
  if (val && val.kbps !== undefined) {
    setText("nq-throughput-val", Number(val.kbps).toFixed(3));
    markGatewayUpdate();
  }
});

onValue(ref(db, "perf/mac"), (snap) => {
  const val = snap.val();
  if (!val) return;
  if (val.network_retries !== undefined) setText("nq-retries-val", val.network_retries);
  if (val.network_cca_busy !== undefined) setText("nq-mac-val", val.network_cca_busy);
  const el = document.getElementById("nq-mac-breakdown");
  if (el) {
    el.innerHTML =
      `<span class="nq-chip">backoffs: ${val.network_backoffs ?? '—'}</span>` +
      `<span class="nq-chip">ACK timeouts: ${val.network_ack_timeouts ?? '—'}</span>` +
      `<span class="nq-chip">drops: ${val.network_drops ?? '—'}</span>` +
      `<span class="nq-chip">dup RX@N4: ${val.dup_rx ?? '—'}</span>`;
  }
});

onValue(ref(db, "perf/mac_nodes"), (snap) => {
  const val = snap.val();
  if (!val) return;
  const retries = document.getElementById("nq-retries-breakdown");
  if (retries) {
    retries.innerHTML = NODE_IDS.map((id) => `<span class="nq-chip">${id}: ${val[id]?.retries ?? '—'}</span>`).join('');
  }
  const duty = document.getElementById("nq-duty-breakdown");
  if (duty) {
    duty.innerHTML = NODE_IDS.map((id) => `<span class="nq-chip">${id}: ${val[id]?.duty_pct ?? '—'}%</span>`).join('');
  }
});

onValue(ref(db, "perf/duty_cycle"), (snap) => {
  const val = snap.val();
  if (val && val.duty_cycle_pct !== undefined) {
    setText("nq-duty-val", Number(val.duty_cycle_pct).toFixed(1));
  }
});

// ---------------------------------------------------------------
// 3. MESH TOPOLOGY & ROUTING (/route/topology) — next hops come from the
//    HELLOs each node broadcasts, paths from the hop trace of delivered packets
// ---------------------------------------------------------------
const NODE_POS = { N1: [60, 50], N2: [60, 150], N3: [160, 100], N4: [260, 100] };
const SVG_NS = "http://www.w3.org/2000/svg";

function drawEdges(topo) {
  const g = document.getElementById("topo-edges");
  if (!g) return;
  while (g.firstChild) g.removeChild(g.firstChild);
  NODE_IDS.forEach((id) => {
    const next = topo[`${id}_next`];
    if (!NODE_POS[next] || !topo[`${id}_alive`]) return;
    const [x1, y1] = NODE_POS[id];
    const [x2, y2] = NODE_POS[next];
    // shorten the line so the arrow head stops at the circle edge
    const dx = x2 - x1, dy = y2 - y1, len = Math.hypot(dx, dy) || 1;
    const r = next === "N4" ? 22 : 18;
    const line = document.createElementNS(SVG_NS, "line");
    line.setAttribute("x1", x1 + dx / len * 16);
    line.setAttribute("y1", y1 + dy / len * 16);
    line.setAttribute("x2", x2 - dx / len * r);
    line.setAttribute("y2", y2 - dy / len * r);
    const direct = (id === "N1" || id === "N2") && next === "N4";
    line.setAttribute("stroke", direct ? "#facc15" : "#00d2ff");
    line.setAttribute("stroke-width", "2");
    line.setAttribute("marker-end", direct ? "url(#arrow-dashed)" : "url(#arrow)");
    g.appendChild(line);
  });
}

onValue(ref(db, "route/topology"), (snap) => {
  const topo = snap.val();
  if (!topo) return;
  markGatewayUpdate();

  NODE_IDS.forEach((id) => {
    const lower = id.toLowerCase();
    const path = topo[`${id}_path`];
    setText(`${lower}-next-hop`, `${topo[`${id}_next`] || "?"}${path ? `  (last path ${path})` : ""}`);
    const badge = document.getElementById(`${lower}-link-badge`);
    if (badge) {
      badge.textContent = topo[`${id}_alive`] ? "ALIVE" : "DEAD";
      badge.className = "topo-badge " + (topo[`${id}_alive`] ? "badge-alive" : "badge-dead");
    }
    const circle = document.getElementById(`node-${lower}-circle`);
    if (circle) circle.setAttribute("stroke-dasharray", topo[`${id}_alive`] ? "none" : "3,3");
  });
  drawEdges(topo);

  const chip = document.getElementById("topo-status-chip");
  if (chip) {
    let mode = "Clustered Multi-Hop";
    if (!topo.N3_alive) mode = "Failover (N3 down, direct to N4)";
    else if (topo.N3_next && topo.N3_next !== "N4") mode = `Alternate route (N3 via ${topo.N3_next})`;
    chip.textContent = `${mode} · scheduler ${topo.scheduler || "N3"}`;
    chip.style.color = mode.startsWith("Clustered") ? "#00d2ff" : "#facc15";
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
    const cum = Number(d.cum_reduction_pct ?? d.reduction_pct ?? 0);
    setText("agg-reduction-val", cum.toFixed(1) + "%");
    const meter = document.getElementById("agg-meter-fill");
    if (meter) meter.style.width = Math.min(100, Math.max(0, cum)) + "%";
    setText("agg-window-red", num(d.reduction_pct) + "%");
    setText("agg-pkt-red", `${num(d.pkt_reduction_pct)}% (${d.pkts_in ?? '—'}→${d.pkts_out ?? '—'})`);

    setText("agg-bytes-in", `${d.bytes_in || 0} B (${(d.n1_count ?? 0) + (d.n2_count ?? 0)} pkts)`);
    setText("agg-bytes-out", `${d.bytes_out || 0} B (1 pkt)`);

    if (d.n1_count > 0) {
      setText("agg-n1-summary", `${d.n1_count}× mean Z ${num(d.n1_mean_z, 2)} | peak ${num(d.n1_max)}`);
    } else {
      setText("agg-n1-summary", "no N1 data in window");
    }
    if (d.n2_count > 0) {
      setText("agg-n2-summary", `${d.n2_count}× ${num(d.n2_flame_max, 0)}/${num(d.n2_flame_min, 0)} (${d.n2_flame_digital === 0 ? "FIRE" : "OK"})`);
    } else {
      setText("agg-n2-summary", "no N2 data in window");
    }

    const irBadge = document.getElementById("agg-ir-badge");
    if (irBadge) {
      const hasObstacle = d.n3_ir === 1;
      irBadge.textContent = hasObstacle ? "OBSTACLE DETECTED" : "CLEAR";
      irBadge.className = "topo-badge " + (hasObstacle ? "badge-dead" : "badge-alive");
    }
    const hops = Number(d.hops || 1);
    setText("agg-hops-chip", `${hops} Hop${hops > 1 ? "s" : ""} to Sink${d.route ? " · " + d.route : ""}`);
  });
});

// ---------------------------------------------------------------
// 5. BLE LOCALIZATION (/ble/N*) — reference positions come from the firmware
// ---------------------------------------------------------------
const BLE_IDS = ["N1", "N2", "N3", "N4"];
const bleNodeData = {};
BLE_IDS.forEach((id) => { bleNodeData[id] = { trueX: null, trueY: null, estX: null, estY: null, error: null }; });

function renderBleMap() {
  const canvas = document.getElementById("ble-map-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  const w = canvas.clientWidth || 300;
  const h = canvas.clientHeight || 200;
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  ctx.clearRect(0, 0, w, h);

  // Auto-scale to all known reference and estimated points
  const pts = [];
  BLE_IDS.forEach((id) => {
    const d = bleNodeData[id];
    if (d.trueX !== null) pts.push([d.trueX, d.trueY]);
    if (d.estX !== null) pts.push([d.estX, d.estY]);
  });
  if (pts.length === 0) return;
  let minX = Math.min(...pts.map((p) => p[0])), maxX = Math.max(...pts.map((p) => p[0]));
  let minY = Math.min(...pts.map((p) => p[1])), maxY = Math.max(...pts.map((p) => p[1]));
  const span = Math.max(maxX - minX, maxY - minY, 1) + 1;
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  minX = cx - span / 2; maxX = cx + span / 2; minY = cy - span / 2; maxY = cy + span / 2;
  const pad = 24;
  const scale = Math.min((w - 2 * pad) / (maxX - minX), (h - 2 * pad) / (maxY - minY));
  const toScreenX = (x) => w / 2 + (x - cx) * scale;
  const toScreenY = (y) => h / 2 - (y - cy) * scale;

  // 0.5 m grid
  ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
  ctx.lineWidth = 1;
  for (let g = Math.floor(minX * 2) / 2; g <= maxX; g += 0.5) {
    ctx.beginPath(); ctx.moveTo(toScreenX(g), 0); ctx.lineTo(toScreenX(g), h); ctx.stroke();
  }
  for (let g = Math.floor(minY * 2) / 2; g <= maxY; g += 0.5) {
    ctx.beginPath(); ctx.moveTo(0, toScreenY(g)); ctx.lineTo(w, toScreenY(g)); ctx.stroke();
  }

  BLE_IDS.forEach((nid) => {
    const data = bleNodeData[nid];
    if (data.trueX === null) return;
    const tx = toScreenX(data.trueX);
    const ty = toScreenY(data.trueY);
    ctx.fillStyle = (nid === "N4") ? "#22c55e" : (nid === "N3") ? "#c084fc" : "#00d2ff";
    ctx.beginPath();
    ctx.moveTo(tx, ty - 6); ctx.lineTo(tx + 6, ty); ctx.lineTo(tx, ty + 6); ctx.lineTo(tx - 6, ty);
    ctx.closePath(); ctx.fill();
    ctx.fillStyle = "#94a3b8";
    ctx.font = "9px 'JetBrains Mono', monospace";
    ctx.fillText(`${nid}`, tx + 8, ty - 2);

    if (data.estX !== null) {
      const ex = toScreenX(data.estX);
      const ey = toScreenY(data.estY);
      ctx.strokeStyle = "rgba(250, 204, 21, 0.4)";
      ctx.setLineDash([2, 2]);
      ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(ex, ey); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#facc15";
      ctx.beginPath(); ctx.arc(ex, ey, 4, 0, Math.PI * 2); ctx.fill();
    }
  });
}

BLE_IDS.forEach((nid) => {
  onValue(ref(db, `ble/${nid}`), (snap) => {
    const data = snap.val();
    if (!data) return;
    const d = bleNodeData[nid];
    d.trueX = Number(data.true_x ?? 0);
    d.trueY = Number(data.true_y ?? 0);
    if (data.localized === false) {
      d.estX = null; d.estY = null; d.error = null;
    } else {
      d.estX = Number(data.est_x ?? 0);
      d.estY = Number(data.est_y ?? 0);
      d.error = Number(data.error_m ?? 0);
    }

    const anchors = [1, 2, 3]
      .filter((i) => data[`a${i}`] && data[`a${i}`] !== "N0")
      .map((i) => `${data[`a${i}`]} ${num(data[`rssi${i}`], 0)} dBm→${num(data[`d${i}`], 2)} m`)
      .join(", ");
    const row = document.getElementById(`ble-${nid.toLowerCase()}-pos`);
    if (row) {
      row.textContent = (data.localized === false)
        ? `proximity only: nearest ${data.nearest} (${anchors})`
        : `est=(${num(data.est_x, 2)}, ${num(data.est_y, 2)}) err=${num(data.error_m, 2)} m · nearest ${data.nearest}`;
      row.title = anchors;
    }
    const localized = BLE_IDS.filter((id) => bleNodeData[id].estX !== null).length;
    setText("ble-anchors-chip", `${localized}/4 nodes localized`);
    renderBleMap();
  });
});

window.addEventListener("resize", renderBleMap);
setTimeout(renderBleMap, 1000);

// ---------------------------------------------------------------
// 6. DYNAMIC TDMA SCHEDULE (/tdma/schedule)
// ---------------------------------------------------------------
onValue(ref(db, "tdma/schedule"), (snap) => {
  const s = snap.val();
  if (!s) return;
  setText("tdma-version-chip", `v${s.version} by ${s.issuer}`);
  setText("tdma-reason", s.reason || "—");
  ["S1", "S2", "S3", "S4"].forEach((slot) => {
    const el = document.getElementById(`tdma-${slot.toLowerCase()}`);
    if (!el) return;
    const owner = s[slot] || "—";
    let role = "";
    if (owner !== "UNUSED") {
      if (slot === "S4" && owner === "N2") role = " RELAY";
      else if (slot === "S4" && owner === "N1") role = " TX (alert extra slot)";
      else role = " TX";
    }
    el.textContent = owner === "UNUSED" ? "UNUSED (radio idle)" : owner + role;
    el.className = "topo-badge " + (owner === "UNUSED" ? "badge-dead" : "badge-alive");
  });
});
