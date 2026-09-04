// Builds the DOM structure for a single node card + returns handles for updating it

export const NODE_CONFIGS = {
  N1: {
    code: "ESP32-NODE-01",
    category: "Telemetry / High-Freq",
    sensorName: "Vibration Sensor",
    datasets: [
      { label: "X-Axis", color: "#00d2ff", glow: "rgba(0, 210, 255, 0.4)" },
      { label: "Y-Axis", color: "#22c55e", glow: "rgba(34, 197, 94, 0.4)" },
      { label: "Z-Axis", color: "#c084fc", glow: "rgba(192, 132, 252, 0.5)" }
    ],
    suggestedMin: -10,
    suggestedMax: 40,
    stepSize: 10,
    maxTicksLimit: 12,
    grace: 0,
    hasAnomaly: false,
    defaultTrend: "—",
    defaultValues: { x: "NA", y: "NA", z: "NA" }
  },
  N2: {
    code: "ESP32-NODE-02",
    category: "Safety / Analog",
    sensorName: "Flame Sensor",
    datasets: [
      { label: "Flame Intensity", color: "#10b981", glow: "rgba(16, 185, 129, 0.4)" }
    ],
    suggestedMin: -500,
    suggestedMax: 5000,
    hasAnomaly: false,
    defaultTrend: "—",
    defaultValues: { single: "NA" },
    additional: "Threshold: 3.0 V"
  },
  N3: {
    code: "ESP32-NODE-03",
    category: "Proximity / Detection",
    sensorName: "IR Sensor",
    datasets: [
      { label: "IR State (0/1)", color: "#38bdf8", glow: "rgba(56, 189, 248, 0.4)" }
    ],
    min: 0,
    max: 1,
    suggestedMin: 0,
    suggestedMax: 1,
    stepSize: 1,
    precision: 0,
    grace: 0,
    tension: 0,
    decimals: 0,
    hasAnomaly: false,
    defaultTrend: "—",
    defaultValues: { single: "NA" },
    additional: "Digital State: 0 or 1"
  }
};

// ==========================================================================
// SLIDING WINDOW Z-SCORE ANOMALY DETECTOR
// Industry-standard approach: rolling baseline (mean + σ) with consecutive
// breach counting. Avoids false positives from single spikes.
//
// States:  NORMAL  →  WARNING (5+ breaches)  →  ALERT (10+ breaches)
// A "breach" = any reading > mean + 2σ of the rolling window.
// When readings return below threshold, the counter decays gradually.
// ==========================================================================
class AnomalyDetector {
  constructor({ windowSize = 30, warnAfter = 5, alertAfter = 10, zThreshold = 2 } = {}) {
    this.windowSize = windowSize;
    this.warnAfter = warnAfter;
    this.alertAfter = alertAfter;
    this.zThreshold = zThreshold;
    this.history = [];           // rolling window of values
    this.breachCount = 0;        // consecutive breach counter
    this.state = 'NORMAL';       // NORMAL | WARNING | ALERT
    this.lastBreachVal = null;
  }

  /** Push a new reading (or array of readings for multi-axis). Returns the new state. */
  push(values) {
    const vals = Array.isArray(values) ? values : [values];
    const magnitude = Math.max(...vals.map(v => Math.abs(Number(v) || 0)));
    this.history.push(magnitude);

    if (this.history.length > this.windowSize) {
      this.history.shift();
    }

    // Need at least 5 samples to compute meaningful statistics
    if (this.history.length < 5) {
      this.state = 'NORMAL';
      return this.state;
    }

    const mean = this.history.reduce((a, b) => a + b, 0) / this.history.length;
    const variance = this.history.reduce((s, v) => s + (v - mean) ** 2, 0) / this.history.length;
    const stdDev = Math.sqrt(variance);

    // Avoid division-by-zero: if stdDev is tiny, use a minimum floor
    const effectiveStd = Math.max(stdDev, mean * 0.05, 0.01);
    const threshold = mean + this.zThreshold * effectiveStd;

    if (magnitude > threshold) {
      this.breachCount++;
      this.lastBreachVal = magnitude;
    } else {
      // Decay breach counter gradually (hysteresis) instead of instant reset
      this.breachCount = Math.max(0, this.breachCount - 1);
    }

    // State escalation
    if (this.breachCount >= this.alertAfter) {
      this.state = 'ALERT';
    } else if (this.breachCount >= this.warnAfter) {
      this.state = 'WARNING';
    } else {
      this.state = 'NORMAL';
    }

    return this.state;
  }

  getInfo() {
    if (this.history.length < 5) return { mean: 0, stdDev: 0, breaches: 0, state: this.state };
    const mean = this.history.reduce((a, b) => a + b, 0) / this.history.length;
    const stdDev = Math.sqrt(this.history.reduce((s, v) => s + (v - mean) ** 2, 0) / this.history.length);
    return { mean: mean.toFixed(2), stdDev: stdDev.toFixed(2), breaches: this.breachCount, state: this.state };
  }
}

export function createNodeCard(container, nodeId) {
  const config = NODE_CONFIGS[nodeId] || {
    code: `ESP32-NODE-${nodeId}`,
    category: "Telemetry / Node",
    sensorName: "Sensor Node",
    datasets: [{ label: "val1", color: "#38bdf8", glow: "rgba(56, 189, 248, 0.4)" }],
    hasAnomaly: false,
    defaultTrend: "→ Stable",
    defaultValues: { single: "—" }
  };

  const card = document.createElement("div");
  card.className = "node-card";
  card.id = `card-${nodeId}`;

  // Build Legend Items HTML
  let legendHtml = "";
  config.datasets.forEach((ds) => {
    let colorClass = "cyan";
    if (ds.label.includes("Y") || ds.label.includes("Flame")) colorClass = "green";
    if (ds.label.includes("Z")) colorClass = "purple";
    legendHtml += `
      <div class="legend-item">
        <span class="legend-box ${colorClass}"></span>
        <span>${ds.label}</span>
      </div>
    `;
  });

  // Build Values HTML
  let valuesHtml = "";
  if (config.datasets.length > 1) {
    valuesHtml = `
      <div class="values-grid" id="values-grid-${nodeId}">
        <div class="val-row"><span>X</span> <span class="val-num" id="val-x-${nodeId}">${config.defaultValues?.x || "—"}</span></div>
        <div class="val-row"><span>Y</span> <span class="val-num" id="val-y-${nodeId}">${config.defaultValues?.y || "—"}</span></div>
        <div class="val-row"><span>Z</span> <span class="val-num" id="val-z-${nodeId}">${config.defaultValues?.z || "—"}</span></div>
      </div>
    `;
  } else {
    valuesHtml = `
      <div class="single-large-val" id="val-single-${nodeId}">${config.defaultValues?.single || "—"}</div>
    `;
  }

  // Cards initialize in INACTIVE / Offline state until Firebase confirms active status
  const statusBadgeHtml = `
    <div class="badge badge-inactive" id="badge-${nodeId}">
      <span class="status-dot"></span>
      <span id="badge-text-${nodeId}">INACTIVE</span>
    </div>
  `;
  const statusSidebarHtml = `
    <div class="status-pill normal" id="status-pill-${nodeId}" style="color: #f87171; border-color: rgba(248, 113, 113, 0.3);">
      <span>✕</span> <span>Offline</span>
    </div>
  `;

  // Trend Class
  let trendClass = "stable";
  if (config.defaultTrend.includes("spike")) trendClass = "spike";
  else if (config.defaultTrend.includes("+")) trendClass = "up";

  // Additional HTML
  let additionalHtml = "";
  if (config.additional) {
    additionalHtml = `
      <div class="sidebar-section">
        <div class="section-label">ADDITIONAL</div>
        <div class="additional-text">${config.additional}</div>
      </div>
    `;
  }

  card.innerHTML = `
    <!-- CARD HEADER -->
    <div class="node-card-header">
      <div class="header-meta">
        <div class="node-code-row">
          <span class="node-code">${config.code}</span>
          <span class="slash">/</span>
          <span class="node-category">${config.category}</span>
        </div>
        <h2 class="sensor-title" id="sensor-title-${nodeId}">${config.sensorName}</h2>
      </div>
      ${statusBadgeHtml}
    </div>

    <!-- CARD BODY: CHART + DOCKED SIDEBAR -->
    <div class="node-card-body">
      <div class="chart-container">
        <canvas id="chart-${nodeId}"></canvas>
      </div>

      <div class="node-sidebar">
        <!-- LEGEND -->
        <div class="sidebar-section">
          <div class="section-label">LEGEND</div>
          <div class="legend-list">
            ${legendHtml}
          </div>
        </div>

        <!-- STATUS -->
        <div class="sidebar-section">
          <div class="section-label">STATUS</div>
          <div id="status-container-${nodeId}">
            ${statusSidebarHtml}
          </div>
        </div>

        <!-- CURRENT VALUES -->
        <div class="sidebar-section">
          <div class="section-label">CURRENT VALUES</div>
          ${valuesHtml}
        </div>

        <!-- RATE / TREND -->
        <div class="sidebar-section">
          <div class="section-label">RATE / TREND</div>
          <div class="trend-val ${trendClass}" id="trend-${nodeId}">${config.defaultTrend}</div>
        </div>

        <!-- ADDITIONAL (OPTIONAL) -->
        ${additionalHtml}
      </div>
    </div>
  `;

  container.appendChild(card);

  // Setup Chart Datasets initialized with a dead line (flatline at 0)
  const INITIAL_POINTS = 30;
  const initialLabels = Array.from({ length: INITIAL_POINTS }, () => "");

  const chartDatasets = config.datasets.map((ds) => ({
    label: ds.label,
    data: new Array(INITIAL_POINTS).fill(0),
    borderColor: ds.color,
    borderWidth: 1.8,
    pointRadius: 0,
    pointHoverRadius: 4,
    tension: config.tension !== undefined ? config.tension : 0.35,
    fill: false
  }));

  const ctx = document.getElementById(`chart-${nodeId}`).getContext("2d");
  const chart = new Chart(ctx, {
    type: "line",
    data: {
      labels: initialLabels,
      datasets: chartDatasets
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      scales: {
        x: {
          display: true,
          ticks: {
            color: "#64748b",
            font: { family: "JetBrains Mono", size: 9 },
            maxRotation: 0,
            autoSkip: true,
            maxTicksLimit: 8
          },
          grid: {
            color: "rgba(255, 255, 255, 0.04)",
            drawBorder: false
          },
          border: { color: "rgba(255, 255, 255, 0.08)" }
        },
        y: {
          display: true,
          grace: config.grace !== undefined ? config.grace : '10%',
          ...(config.min !== undefined ? { min: config.min } : {}),
          ...(config.max !== undefined ? { max: config.max } : {}),
          suggestedMin: config.suggestedMin ?? 0,
          suggestedMax: config.suggestedMax ?? 100,
          ticks: {
            color: "#64748b",
            font: { family: "JetBrains Mono", size: 9 },
            ...(config.stepSize !== undefined ? { stepSize: config.stepSize } : {}),
            ...(config.precision !== undefined ? { precision: config.precision } : {}),
            maxTicksLimit: config.maxTicksLimit ?? (config.stepSize ? 12 : 5),
            callback: function(value) {
              if (config.stepSize === 1 && config.max === 1) {
                if (value === 0 || value === 1) return value;
                return '';
              }
              return value;
            }
          },
          grid: {
            color: "rgba(255, 255, 255, 0.04)",
            drawBorder: false
          },
          border: { color: "rgba(255, 255, 255, 0.08)" }
        }
      },
      plugins: {
        legend: { display: false },
        tooltip: {
          enabled: true,
          backgroundColor: "#0d1424",
          borderColor: "rgba(56, 139, 253, 0.3)",
          borderWidth: 1,
          titleFont: { family: "JetBrains Mono", size: 10 },
          bodyFont: { family: "JetBrains Mono", size: 11 }
        },
      }
    }
  });

  // Anomaly detector instance for this node
  const detector = new AnomalyDetector({ windowSize: 30, warnAfter: 5, alertAfter: 10, zThreshold: 2 });

  // Helper: update sidebar status based on anomaly detector state
  // Active state tracker for this node
  let isNodeActive = false;
  let latestValues = null;

  function renderValues({ seq, v1, v2, v3, trendText }) {
    if (config.datasets.length > 1) {
      const xEl = document.getElementById(`val-x-${nodeId}`);
      const yEl = document.getElementById(`val-y-${nodeId}`);
      const zEl = document.getElementById(`val-z-${nodeId}`);
      if (xEl && v1 !== undefined) xEl.textContent = `${typeof v1 === 'number' ? v1.toFixed(1) : v1}g`;
      if (yEl && v2 !== undefined) yEl.textContent = `${typeof v2 === 'number' ? v2.toFixed(1) : v2}g`;
      if (zEl && v3 !== undefined) zEl.textContent = `${typeof v3 === 'number' ? v3.toFixed(1) : v3}g`;
    } else {
      const sEl = document.getElementById(`val-single-${nodeId}`);
      if (sEl && v1 !== undefined) {
        let formatted = v1;
        if (typeof v1 === 'number') {
          if (config.decimals !== undefined) {
            formatted = v1.toFixed(config.decimals);
          } else if (Number.isInteger(v1)) {
            formatted = v1;
          } else {
            formatted = v1.toFixed(1);
          }
        }
        sEl.textContent = formatted;
      }
    }

    if (trendText) {
      const trendEl = document.getElementById(`trend-${nodeId}`);
      if (trendEl) {
        trendEl.textContent = trendText;
        trendEl.className = "trend-val " + (trendText.includes("+") ? "up" : "stable");
      }
    }
  }

  // Helper: update sidebar status based on anomaly detector state
  function updateAnomalyStatus(state) {
    if (!isNodeActive) return; // Never mark active if node is offline in Firebase
    const badge = document.getElementById(`badge-${nodeId}`);
    const statusContainer = document.getElementById(`status-container-${nodeId}`);
    if (!badge || !statusContainer) return;

    if (state === 'ALERT') {
      const info = detector.getInfo();
      badge.className = "badge badge-anomaly";
      badge.innerHTML = `<span class="anomaly-icon">✦</span><span>ALERT</span>`;
      statusContainer.innerHTML = `
        <div class="status-pill anomaly" style="background: rgba(239, 68, 68, 0.15); border-color: rgba(239, 68, 68, 0.5); color: #f87171;">
          <span>🔴</span> <span>SUSTAINED ANOMALY</span>
        </div>
        <div class="alert-message" style="color: #f87171;">${info.breaches} consecutive breaches (μ=${info.mean}, σ=${info.stdDev})</div>
      `;
    } else if (state === 'WARNING') {
      const info = detector.getInfo();
      badge.className = "badge badge-warning";
      badge.innerHTML = `<span class="anomaly-icon">⚠</span><span>WARNING</span>`;
      statusContainer.innerHTML = `
        <div class="status-pill anomaly">
          <span>⚠</span> <span>DISTURBANCE DETECTED</span>
        </div>
        <div class="alert-message">${info.breaches} breaches in window (μ=${info.mean}, σ=${info.stdDev})</div>
      `;
    } else {
      badge.className = "badge badge-active";
      badge.innerHTML = `<span class="status-dot"></span><span>ACTIVE</span>`;
      statusContainer.innerHTML = `
        <div class="status-pill normal"><span>✓</span> <span>Normal</span></div>
      `;
    }
  }

  return {
    chart,
    config,
    detector,
    isActive() {
      return isNodeActive;
    },
    pushDeadPoint() {
      const MAX_POINTS = 30;
      const timeStr = new Date().toLocaleTimeString('en-GB');
      chart.data.labels.push(timeStr);
      if (chart.data.labels.length > MAX_POINTS) {
        chart.data.labels.shift();
      }

      chart.data.datasets.forEach((ds) => {
        ds.data.push(0);
        if (ds.data.length > MAX_POINTS) {
          ds.data.shift();
        }
      });

      chart.update("none");
    },
    resetToDeadLine() {
      const MAX_POINTS = 30;
      chart.data.labels = Array.from({ length: MAX_POINTS }, () => "");
      chart.data.datasets.forEach((ds) => {
        ds.data = new Array(MAX_POINTS).fill(0);
      });
      chart.update("none");
    },
    setStatus(isActive, isAnomaly = false) {
      isNodeActive = Boolean(isActive);
      const badge = document.getElementById(`badge-${nodeId}`);
      const statusContainer = document.getElementById(`status-container-${nodeId}`);
      if (!badge || !statusContainer) return;

      if (!isNodeActive) {
        badge.className = "badge badge-inactive";
        badge.innerHTML = `<span class="status-dot"></span><span>INACTIVE</span>`;
        statusContainer.innerHTML = `
          <div class="status-pill normal" style="color: #f87171; border-color: rgba(248, 113, 113, 0.3);">
            <span>✕</span> <span>Offline</span>
          </div>
        `;
        // Clear all sidebar values to NA when inactive
        if (config.datasets.length > 1) {
          const xEl = document.getElementById(`val-x-${nodeId}`);
          const yEl = document.getElementById(`val-y-${nodeId}`);
          const zEl = document.getElementById(`val-z-${nodeId}`);
          if (xEl) xEl.textContent = 'NA';
          if (yEl) yEl.textContent = 'NA';
          if (zEl) zEl.textContent = 'NA';
        } else {
          const sEl = document.getElementById(`val-single-${nodeId}`);
          if (sEl) sEl.textContent = 'NA';
        }
        const trendEl = document.getElementById(`trend-${nodeId}`);
        if (trendEl) {
          trendEl.textContent = '—';
          trendEl.className = 'trend-val stable';
        }
      } else if (isAnomaly) {
        // Anomaly state is now handled by the detector in pushChartPoint
        updateAnomalyStatus(detector.state !== 'NORMAL' ? detector.state : 'WARNING');
        if (latestValues) renderValues(latestValues);
      } else {
        // Only set to active/normal if detector agrees
        updateAnomalyStatus(detector.state);
        if (latestValues) renderValues(latestValues);
      }
    },
    setSensorType(type) {
      const titleEl = document.getElementById(`sensor-title-${nodeId}`);
      if (titleEl && type) {
        titleEl.textContent = type;
      }
    },
    setLatestValues(seq, v1, v2, v3, trendText) {
      latestValues = { seq, v1, v2, v3, trendText };
      if (!isNodeActive) return;
      renderValues(latestValues);
    },
    pushChartPoint(label, v1, v2, v3) {
      const MAX_POINTS = 30;
      chart.data.labels.push(label || "");

      if (config.datasets.length === 1) {
        chart.data.datasets[0].data.push(v1);
        if (chart.data.datasets[0].data.length > MAX_POINTS) {
          chart.data.datasets[0].data.shift();
        }
        // Feed single value to anomaly detector
        detector.push(v1);
      } else {
        const vals = [v1, v2, v3];
        chart.data.datasets.forEach((ds, idx) => {
          const val = vals[idx] !== undefined ? vals[idx] : v1;
          ds.data.push(val);
          if (ds.data.length > MAX_POINTS) {
            ds.data.shift();
          }
        });
        // Feed all axis values to anomaly detector
        detector.push([v1, v2, v3].filter(v => v !== undefined));
      }

      if (chart.data.labels.length > MAX_POINTS) {
        chart.data.labels.shift();
      }

      chart.update("none");

      // Auto-update sidebar anomaly status from detector only when online
      if (isNodeActive) {
        updateAnomalyStatus(detector.state);
      }
    }
  };
}