import { db } from "./firebase-config.js";
import {
  ref,
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