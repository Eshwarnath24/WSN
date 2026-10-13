// ================= NODE 4 (Gateway / Coordinator) - central_node.ino =================
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

const char* ssid = "iQOO Neo6";
const char* password = "eshwar24";
const char* firebaseHost = "https://wsn-project-bb012-default-rtdb.asia-southeast1.firebasedatabase.app";
const int WIFI_TIMEOUT_ATTEMPTS = 20;
const unsigned long NODE_TIMEOUT_MS = 5000;

bool wifiConnected = false;

// Persistent HTTP client & secure connection for HTTP Keep-Alive (reduces request time from ~2s to ~80ms)
WiFiClientSecure fbClient;
HTTPClient fbHttp;

#define MSG_REGISTER      0
#define MSG_REGISTER_ACK  1
#define MSG_SENSOR_DATA   2

typedef struct struct_message {
  uint8_t  msg_type;
  char     node_id[4];
  char     sensor_type[10];
  float    val1;
  float    val2;
  float    val3;
  unsigned long seq_num;
  unsigned long timestamp;
  unsigned long ack_delay_ms;
} struct_message;

#define MAX_NODES 4
#define QUEUE_SIZE 4

struct NodeTracker {
  String nodeId;
  String sensorType;
  String role;

  unsigned long lastSeen;
  bool isActive;
  bool everSeen;
  unsigned long wentInactiveAt;

  // Ring buffer queue for incoming ESP-NOW messages
  struct_message queueData[QUEUE_SIZE];
  int queueRssi[QUEUE_SIZE];
  volatile uint8_t queueHead;
  volatile uint8_t queueTail;
  volatile uint8_t queueCount;

  uint8_t peerMac[6];
  volatile bool pendingRegister;
  bool registered;

  long firstSeq;
  long highestSeq;
  long receivedUniqueCount;
  long duplicateCount;
  long outOfOrderCount;

  int lastRssi;
  unsigned long lastAckDelay;
  unsigned long lastQosPush;
};

NodeTracker nodes[MAX_NODES];

void initNodeTable() {
  const char* ids[MAX_NODES]          = {"N1", "N2", "N3", "N4"};
  const char* roles[MAX_NODES]        = {"Member", "Member", "Member", "Coordinator"};
  const char* sensorTypes[MAX_NODES]  = {"vibration", "flame", "ir", ""};
  for (int i = 0; i < MAX_NODES; i++) {
    nodes[i].nodeId = ids[i];
    nodes[i].role = roles[i];
    nodes[i].sensorType = sensorTypes[i];
    nodes[i].lastSeen = 0;
    nodes[i].isActive = false;
    nodes[i].everSeen = false;
    nodes[i].wentInactiveAt = 0;
    nodes[i].queueHead = 0;
    nodes[i].queueTail = 0;
    nodes[i].queueCount = 0;
    nodes[i].pendingRegister = false;
    nodes[i].registered = false;
    nodes[i].firstSeq = -1;
    nodes[i].highestSeq = -1;
    nodes[i].receivedUniqueCount = 0;
    nodes[i].duplicateCount = 0;
    nodes[i].outOfOrderCount = 0;
    nodes[i].lastRssi = 0;
    nodes[i].lastAckDelay = 0;
    nodes[i].lastQosPush = 0;
  }
}

int findNodeIndex(String nodeId) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].nodeId == nodeId) return i;
  }
  return -1;
}

void connectWiFi() {
  WiFi.disconnect(true);
  delay(100);
  Serial.println("\n[WiFi] Connecting to: " + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < WIFI_TIMEOUT_ATTEMPTS) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    fbClient.stop(); // reset any previous connection
    Serial.println("\n[WiFi] CONNECTED");
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Channel: ");
    Serial.println(WiFi.channel());
  } else {
    wifiConnected = false;
    Serial.print("\n[WiFi] FAILED to connect. Status code: ");
    Serial.println(WiFi.status());
  }
}

// ---------------- Fast Reusable Firebase Helper ----------------

bool sendFirebaseRequest(const String& method, const String& path, const String& payload) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    return false;
  }

  String url = String(firebaseHost) + path;

  // Up to 2 attempts: if stale keep-alive connection dropped, reconnect once
  for (int attempt = 0; attempt < 2; attempt++) {
    fbHttp.begin(fbClient, url);
    fbHttp.setReuse(true);       // Enable HTTP/1.1 Keep-Alive (reuses TLS session)
    fbHttp.setTimeout(1500);     // Non-blocking 1.5s timeout
    fbHttp.addHeader("Content-Type", "application/json");

    int httpCode = 0;
    if (method == "POST") {
      httpCode = fbHttp.POST(payload);
    } else if (method == "PUT") {
      httpCode = fbHttp.PUT(payload);
    }

    if (httpCode > 0) {
      fbHttp.end();
      return true;
    }

    // Attempt failed (e.g. idle keep-alive closed by Firebase server)
    fbHttp.end();
    fbClient.stop();
    if (attempt == 0) {
      delay(20);
    }
  }

  Serial.printf("[Firebase] %s %s FAILED\n", method.c_str(), path.c_str());
  return false;
}

// ---------------- Firebase Operations ----------------

void setNodeStatus(String nodeId, String sensorType, String role, bool active) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    Serial.println("[Firebase] Status update SKIPPED - WiFi not connected");
    return;
  }

  // 1. Set status attribute directly under /nodes/{nodeId}/status ("active" / "inactive")
  sendFirebaseRequest("PUT", "/nodes/" + nodeId + "/status.json", active ? "\"active\"" : "\"inactive\"");

  // 2. Set sensor_type attribute directly under /nodes/{nodeId}/sensor_type ("vibration" / "flame" / "ir")
  if (sensorType.length() > 0) {
    sendFirebaseRequest("PUT", "/nodes/" + nodeId + "/sensor_type.json", "\"" + sensorType + "\"");
  }

  // 3. Set role attribute directly under /nodes/{nodeId}/role ("Member" / "Coordinator")
  if (role.length() > 0) {
    sendFirebaseRequest("PUT", "/nodes/" + nodeId + "/role.json", "\"" + role + "\"");
  }

  Serial.print("[Firebase] Status set [");
  Serial.print(nodeId);
  Serial.print("] -> ");
  Serial.print(active ? "ACTIVE" : "INACTIVE");
  if (sensorType.length() > 0) {
    Serial.print(" | sensor_type: ");
    Serial.print(sensorType);
  }
  Serial.println();
}

void pushReading(String nodeId, String sensorType, float v1, float v2, float v3,
                  unsigned long seq, unsigned long srcTimestamp, int rssi,
                  unsigned long ackDelay, bool isDup, bool isOOO) {
  String path = "/nodes/" + nodeId + "/data.json";

  String payload = "{";
  payload += "\"node_id\":\"" + nodeId + "\",";
  payload += "\"sensor_type\":\"" + sensorType + "\",";
  payload += "\"seq_num\":" + String(seq) + ",";
  payload += "\"timestamp\":" + String(srcTimestamp) + ",";
  payload += "\"val1\":" + String(v1) + ",";
  payload += "\"val2\":" + String(v2) + ",";
  payload += "\"val3\":" + String(v3) + ",";
  payload += "\"rssi_dbm\":" + String(rssi) + ",";
  payload += "\"send_ack_delay_ms\":" + String(ackDelay) + ",";
  payload += "\"is_duplicate\":" + String(isDup ? "true" : "false") + ",";
  payload += "\"is_out_of_order\":" + String(isOOO ? "true" : "false");
  payload += "}";

  if (sendFirebaseRequest("POST", path, payload)) {
    Serial.print("[Firebase] Pushed [");
    Serial.print(nodeId);
    Serial.print("] seq=");
    Serial.println(seq);
  }
}

void pushQos(String nodeId, long expected, long received, long lost, float pdr,
             long dupCount, long oooCount, int rssi, unsigned long ackDelay) {
  String path = "/nodes/" + nodeId + "/qos.json";

  String payload = "{";
  payload += "\"expected\":" + String(expected) + ",";
  payload += "\"received\":" + String(received) + ",";
  payload += "\"lost\":" + String(lost) + ",";
  payload += "\"pdr\":" + String(pdr, 2) + ",";
  payload += "\"duplicates\":" + String(dupCount) + ",";
  payload += "\"out_of_order\":" + String(oooCount) + ",";
  payload += "\"rssi_dbm\":" + String(rssi) + ",";
  payload += "\"last_ack_delay_ms\":" + String(ackDelay);
  payload += "}";

  sendFirebaseRequest("PUT", path, payload);
}

void logEvent(String nodeId, String eventType, String detail) {
  Serial.print("[Event] "); Serial.print(nodeId); Serial.print(" - ");
  Serial.print(eventType); Serial.print(" - "); Serial.println(detail);

  String payload = "{";
  payload += "\"node_id\":\"" + nodeId + "\",";
  payload += "\"event\":\"" + eventType + "\",";
  payload += "\"detail\":\"" + detail + "\",";
  payload += "\"timestamp\":" + String(millis());
  payload += "}";

  sendFirebaseRequest("POST", "/events.json", payload);
}

// ---------------- ESP-NOW receive callback ----------------
// Lightweight queue enqueue - packets are preserved even during active Wi-Fi upload

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  struct_message temp;
  memcpy(&temp, data, sizeof(temp));

  String nodeId = String(temp.node_id);
  int idx = findNodeIndex(nodeId);
  if (idx == -1) return;

  if (temp.msg_type == MSG_REGISTER) {
    memcpy(nodes[idx].peerMac, info->src_addr, 6);
    nodes[idx].pendingRegister = true;
    return;
  }

  if (temp.msg_type == MSG_SENSOR_DATA) {
    nodes[idx].lastSeen = millis();
    Serial.print("[ESP-NOW] Received from ");
    Serial.print(nodeId);
    Serial.print(" | seq=");
    Serial.println(temp.seq_num);
    if (nodes[idx].queueCount < QUEUE_SIZE) {
      nodes[idx].queueData[nodes[idx].queueHead] = temp;
      nodes[idx].queueRssi[nodes[idx].queueHead] = info->rx_ctrl->rssi;
      nodes[idx].queueHead = (nodes[idx].queueHead + 1) % QUEUE_SIZE;
      nodes[idx].queueCount++;
    } else {
      // Overwrite oldest if ring buffer is full
      nodes[idx].queueData[nodes[idx].queueHead] = temp;
      nodes[idx].queueRssi[nodes[idx].queueHead] = info->rx_ctrl->rssi;
      nodes[idx].queueHead = (nodes[idx].queueHead + 1) % QUEUE_SIZE;
      nodes[idx].queueTail = (nodes[idx].queueTail + 1) % QUEUE_SIZE;
    }
  }
}

// ---------------- Setup ----------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 4 (Gateway) BOOTING =====");

  initNodeTable();
  connectWiFi();

  fbClient.setInsecure(); // Disable certificate validation for fast connection
  fbClient.setTimeout(2); // 2 second TLS connection timeout

  Serial.println("[ESP-NOW] Initializing...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] INIT FAILED - halting");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("[Startup] Resetting all node statuses to inactive...");
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].nodeId != "N4") {
      setNodeStatus(nodes[i].nodeId, nodes[i].sensorType, nodes[i].role, false);
    }
  }

  Serial.println("[ESP-NOW] Ready. Waiting for registrations and sensor data...");
  Serial.println("===== SETUP COMPLETE =====\n");
}

// ---------------- Main loop ----------------

void loop() {
  // ---- 1. Handle pending REGISTER requests ----
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].pendingRegister) {
      nodes[i].pendingRegister = false;

      if (!esp_now_is_peer_exist(nodes[i].peerMac)) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, nodes[i].peerMac, 6);
        peerInfo.channel = 0;   // use current channel
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
      }

      bool firstTimeRegistering = !nodes[i].registered;
      nodes[i].registered = true;

      struct_message ack = {};
      ack.msg_type = MSG_REGISTER_ACK;
      strcpy(ack.node_id, nodes[i].nodeId.c_str());
      esp_now_send(nodes[i].peerMac, (uint8_t *)&ack, sizeof(ack));

      if (firstTimeRegistering) {
        logEvent(nodes[i].nodeId, "REGISTERED", "Node joined the cluster (" + nodes[i].role + ")");
      }
    }
  }

  // ---- 2. Process queued sensor readings ----
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].queueCount > 0) {
      struct_message d = nodes[i].queueData[nodes[i].queueTail];
      int rssi = nodes[i].queueRssi[nodes[i].queueTail];
      nodes[i].queueTail = (nodes[i].queueTail + 1) % QUEUE_SIZE;
      nodes[i].queueCount--;

      bool wasInactive = !nodes[i].isActive;
      nodes[i].isActive = true;
      nodes[i].sensorType = String(d.sensor_type);

      if (wasInactive || !nodes[i].everSeen) {
        if (nodes[i].everSeen) {
          unsigned long recovery = millis() - nodes[i].wentInactiveAt;
          logEvent(nodes[i].nodeId, "RECOVERED", "Back online after " + String(recovery) + " ms");
        } else {
          logEvent(nodes[i].nodeId, "ONLINE", "First data received from node");
        }
        nodes[i].everSeen = true;
        setNodeStatus(nodes[i].nodeId, nodes[i].sensorType, nodes[i].role, true);
      }

      // ---- Sequence analysis: duplicate / out-of-order / loss ----
      long seq = (long)d.seq_num;
      bool isDuplicate = false;
      bool isOutOfOrder = false;

      if (nodes[i].firstSeq == -1) {
        nodes[i].firstSeq = seq;
        nodes[i].highestSeq = seq;
        nodes[i].receivedUniqueCount = 1;
      } else if (seq == nodes[i].highestSeq) {
        isDuplicate = true;
        nodes[i].duplicateCount++;
      } else if (seq < nodes[i].highestSeq) {
        isOutOfOrder = true;
        nodes[i].outOfOrderCount++;
        nodes[i].receivedUniqueCount++;
      } else {
        nodes[i].highestSeq = seq;
        nodes[i].receivedUniqueCount++;
      }

      long expected = nodes[i].highestSeq - nodes[i].firstSeq + 1;
      long lost = expected - nodes[i].receivedUniqueCount;
      if (lost < 0) lost = 0;
      float pdr = (expected > 0) ? (nodes[i].receivedUniqueCount * 100.0 / expected) : 100.0;

      nodes[i].lastRssi = rssi;
      nodes[i].lastAckDelay = d.ack_delay_ms;

      if (isDuplicate) {
        Serial.print("[QoS] "); Serial.print(nodes[i].nodeId);
        Serial.print(" DUPLICATE seq="); Serial.println(seq);
      }
      if (isOutOfOrder) {
        Serial.print("[QoS] "); Serial.print(nodes[i].nodeId);
        Serial.print(" OUT-OF-ORDER seq="); Serial.println(seq);
      }

      // 1. Immediately push reading to Firebase RTDB
      pushReading(nodes[i].nodeId, nodes[i].sensorType, d.val1, d.val2, d.val3,
                  d.seq_num, d.timestamp, rssi, d.ack_delay_ms, isDuplicate, isOutOfOrder);

      // 2. Push QoS periodically (every 2 seconds) or immediately on sequence anomalies
      // Avoids doubling HTTP traffic on every single packet
      if (millis() - nodes[i].lastQosPush >= 2000 || isDuplicate || isOutOfOrder) {
        nodes[i].lastQosPush = millis();
        pushQos(nodes[i].nodeId, expected, nodes[i].receivedUniqueCount, lost, pdr,
                nodes[i].duplicateCount, nodes[i].outOfOrderCount, rssi, d.ack_delay_ms);
      }
    }
  }

  // ---- 3. Timeout check -> mark inactive, log event, record time for recovery calc ----
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].everSeen && nodes[i].isActive) {
      if (millis() - nodes[i].lastSeen > NODE_TIMEOUT_MS) {
        nodes[i].isActive = false;
        nodes[i].wentInactiveAt = millis();
        Serial.print("[Node Monitor] ");
        Serial.print(nodes[i].nodeId);
        Serial.println(" TIMED OUT - marking inactive");
        setNodeStatus(nodes[i].nodeId, nodes[i].sensorType, nodes[i].role, false);
        logEvent(nodes[i].nodeId, "OFFLINE",
                 "No data for " + String(NODE_TIMEOUT_MS) + " ms - marked inactive");
      }
    }
  }

  // ---- 4. WiFi watchdog ----
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 10000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED && wifiConnected) {
      Serial.println("[WiFi] Connection LOST. Will retry...");
      wifiConnected = false;
      fbClient.stop();
    } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
      Serial.println("[WiFi] Reconnected!");
      wifiConnected = true;
      fbClient.stop();
    } else if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
  }
}