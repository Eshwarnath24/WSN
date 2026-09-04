// ================= NODE 4 (Gateway) - central_node.ino =================
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

typedef struct struct_message {
  char node_id[4];
  char sensor_type[10];
  float val1;
  float val2;
  float val3;
  int seq_num;
} struct_message;

#define MAX_NODES 4
struct NodeTracker {
  String nodeId;
  String sensorType;
  unsigned long lastSeen;
  bool isActive;
  bool everSeen;
  struct_message pendingData;
  volatile bool hasPending;
};

NodeTracker nodes[MAX_NODES] = {
  {"N1", "", 0, false, false, {}, false},
  {"N2", "", 0, false, false, {}, false},
  {"N3", "", 0, false, false, {}, false},
  {"N4", "", 0, false, false, {}, false}
};

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

void setNodeStatus(String nodeId, String sensorType, bool active) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    Serial.println("[Firebase] Status update SKIPPED - WiFi not connected");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  http.begin(client, String(firebaseHost) + "/nodes/" + nodeId + "/status.json");
  http.addHeader("Content-Type", "application/json");
  http.PUT(active ? "\"active\"" : "\"inactive\"");
  http.end();

  if (sensorType.length() > 0) {
    http.begin(client, String(firebaseHost) + "/nodes/" + nodeId + "/sensor_type.json");
    http.addHeader("Content-Type", "application/json");
    http.PUT("\"" + sensorType + "\"");
    http.end();
  }

  Serial.print("[Firebase] Status set [");
  Serial.print(nodeId);
  Serial.print("] -> ");
  Serial.println(active ? "ACTIVE" : "INACTIVE");
}

void pushReading(String nodeId, float v1, float v2, float v3, int seq) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    Serial.println("[Firebase] SKIPPED - WiFi not connected");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String url = String(firebaseHost) + "/nodes/" + nodeId + "/data.json";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  String payload = "{";
  payload += "\"seq_num\":" + String(seq) + ",";
  payload += "\"timestamp\":" + String(millis()) + ",";
  payload += "\"val1\":" + String(v1) + ",";
  payload += "\"val2\":" + String(v2) + ",";
  payload += "\"val3\":" + String(v3);
  payload += "}";

  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    Serial.print("[Firebase] Pushed [");
    Serial.print(nodeId);
    Serial.print("] -> HTTP ");
    Serial.println(httpCode);
  } else {
    Serial.print("[Firebase] Push FAILED [");
    Serial.print(nodeId);
    Serial.print("] -> Error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

// ---- Callback fires INSTANTLY on packet arrival, independent of loop() blocking ----
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  struct_message temp;
  memcpy(&temp, data, sizeof(temp));

  String nodeId = String(temp.node_id);
  int idx = findNodeIndex(nodeId);
  if (idx == -1) return;

  // Record lastSeen IMMEDIATELY - this is the actual fix
  nodes[idx].lastSeen = millis();
  nodes[idx].pendingData = temp;
  nodes[idx].hasPending = true;

  Serial.print("[ESP-NOW] From: ");
  Serial.print(temp.node_id);
  Serial.print(" | Type: ");
  Serial.print(temp.sensor_type);
  Serial.print(" | Val1: ");
  Serial.print(temp.val1);
  Serial.print(" | Val2: ");
  Serial.print(temp.val2);
  Serial.print(" | Val3: ");
  Serial.print(temp.val3);
  Serial.print(" | Seq: ");
  Serial.println(temp.seq_num);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 4 (Gateway) BOOTING =====");

  connectWiFi();

  Serial.println("[ESP-NOW] Initializing...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] INIT FAILED - halting");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // ---- NEW: reset all known nodes to inactive at boot, clearing stale Firebase state ----
  Serial.println("[Startup] Resetting all node statuses to inactive...");
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].nodeId != "N4") {  // N4 is the gateway itself, skip
      setNodeStatus(nodes[i].nodeId, "", false);
    }
  }

  Serial.println("[ESP-NOW] Ready. Waiting for sensor data...");
  Serial.println("===== SETUP COMPLETE =====\n");
}

void loop() {
  // Process each node's pending data independently - no shared buffer collision
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].hasPending) {
      nodes[i].hasPending = false;
      struct_message d = nodes[i].pendingData;

      bool wasInactive = !nodes[i].isActive;
      nodes[i].isActive = true;
      nodes[i].sensorType = String(d.sensor_type);

      if (wasInactive || !nodes[i].everSeen) {
        nodes[i].everSeen = true;
        setNodeStatus(nodes[i].nodeId, nodes[i].sensorType, true);
      }

      pushReading(nodes[i].nodeId, d.val1, d.val2, d.val3, d.seq_num);
    }
  }

  // Timeout check now uses lastSeen set directly by the callback - accurate regardless of upload speed
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].everSeen && nodes[i].isActive) {
      if (millis() - nodes[i].lastSeen > NODE_TIMEOUT_MS) {
        nodes[i].isActive = false;
        Serial.print("[Node Monitor] ");
        Serial.print(nodes[i].nodeId);
        Serial.println(" TIMED OUT - marking inactive");
        setNodeStatus(nodes[i].nodeId, "", false);
      }
    }
  }

  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 10000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED && wifiConnected) {
      Serial.println("[WiFi] Connection LOST. Will retry...");
      wifiConnected = false;
    } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
      Serial.println("[WiFi] Reconnected!");
      wifiConnected = true;
    } else if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
  }
}