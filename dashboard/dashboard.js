import { db } from "./firebase-config.js";
import { ref, onValue, query, limitToLast } from "https://www.gstatic.com/firebasejs/10.12.0/firebase-database.js";
import { createNodeCard } from "./node-card.js";

const NODE_IDS = ["N1", "N2", "N3"];
const container = document.getElementById("grid");
const cards = {};
let totalPackets = 0;

// Initialize Cards
NODE_IDS.forEach((nodeId) => {
  cards[nodeId] = createNodeCard(container, nodeId);
});

// Real-time Firebase listeners — all data is dynamic
NODE_IDS.forEach((nodeId) => {
  // Listen for status changes
  onValue(ref(db, `nodes/${nodeId}/status`), (snapshot) => {
    const status = snapshot.val();
    cards[nodeId].setStatus(status === "active");
    updateActiveCount();
  });

  // Listen for sensor type
  onValue(ref(db, `nodes/${nodeId}/sensor_type`), (snapshot) => {
    const type = snapshot.val();
    if (type) {
      cards[nodeId].setSensorType(type);
    }
  });

  // Listen for the last 30 readings, live-updating
  const dataQuery = query(ref(db, `nodes/${nodeId}/data`), limitToLast(30));
  let lastKeys = new Set();

  onValue(dataQuery, (snapshot) => {
    if (!snapshot.exists()) return;

    const entries = [];
    snapshot.forEach((child) => {
      entries.push({ key: child.key, ...child.val() });
    });

    entries.forEach((entry) => {
      if (!lastKeys.has(entry.key)) {
        lastKeys.add(entry.key);
        const v1 = Number(entry.val1 ?? 0);
        const v2 = Number(entry.val2 ?? entry.val1 ?? 0);
        const v3 = Number(entry.val3 ?? entry.val1 ?? 0);

        cards[nodeId].pushChartPoint(entry.seq_num, v1, v2, v3);
        cards[nodeId].setLatestValues(entry.seq_num, v1, v2, v3);

        totalPackets++;
        document.getElementById("total-packets").textContent = totalPackets.toLocaleString();
      }
    });
  });
});




function updateActiveCount() {
  let activeCount = 0;
  NODE_IDS.forEach((id) => {
    const cardEl = document.getElementById(`card-${id}`);
    const badge = cardEl ? cardEl.querySelector(".badge-active, .badge-anomaly, .badge-warning") : null;
    if (badge) activeCount++;
  });
  const countEl = document.getElementById("active-count");
  if (countEl) countEl.textContent = `${activeCount}/${NODE_IDS.length}`;
}