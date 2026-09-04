// ================= NODE 3 (IR Sensor) - SENDER =================
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

uint8_t receiverMAC[] = {0x8C, 0x94, 0xDF, 0x7C, 0x3F, 0x58}; // N4 gateway MAC
#define N4_CHANNEL 11  // must match N4's printed channel

#define IR_DO_PIN 4    // confirmed working - digital only, active-low (1=clear, 0=blocked)
#define DEBOUNCE_MS 50 // ignore state flips faster than this (comparator chatter guard)

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
  unsigned long timestamp;      // sender's own millis() at moment of send
  unsigned long ack_delay_ms;   // send->ack delay measured for the PREVIOUS packet
} struct_message;

struct_message outData;
unsigned long seqCounter = 0;

volatile bool registered = false;
volatile bool ackReceivedFlag = false;
unsigned long lastSendMillis = 0;
volatile unsigned long pendingAckDelay = 0;

// ---- Debounce state ----
int lastRawReading = HIGH;      // last raw sample (unfiltered)
int stableState = HIGH;         // debounced, trusted state (HIGH=clear, LOW=blocked)
unsigned long lastChangeTime = 0;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  unsigned long now = millis();
  if (status == ESP_NOW_SEND_SUCCESS) {
    pendingAckDelay = now - lastSendMillis;
  }
  Serial.print("[ESP-NOW] Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  struct_message temp;
  memcpy(&temp, data, sizeof(temp));
  if (temp.msg_type == MSG_REGISTER_ACK && String(temp.node_id) == "N3") {
    registered = true;
    ackReceivedFlag = true;
    Serial.println("[Register] REGISTER_ACK received from N4");
  }
}

void sendPacket(uint8_t type) {
  outData.msg_type = type;
  strcpy(outData.node_id, "N3");
  strcpy(outData.sensor_type, "ir");
  outData.ack_delay_ms = pendingAckDelay;   // delay measured for the previous send
  lastSendMillis = millis();
  outData.timestamp = lastSendMillis;
  esp_now_send(receiverMAC, (uint8_t *)&outData, sizeof(outData));
}

void registerWithGateway() {
  Serial.println("[Register] Sending REGISTER to N4...");
  outData.val1 = 0; outData.val2 = 0; outData.val3 = 0;
  outData.seq_num = 0;

  int attempts = 0;
  while (!registered && attempts < 10) {
    ackReceivedFlag = false;   // reset before each attempt, avoids stale-flag ambiguity
    sendPacket(MSG_REGISTER);
    unsigned long waitStart = millis();
    while (!ackReceivedFlag && millis() - waitStart < 1000) {
      delay(10);
    }
    if (registered) break;
    attempts++;
    Serial.println("[Register] No ACK yet, retrying...");
  }

  if (registered) {
    Serial.println("[Register] N3 REGISTERED with gateway.");
  } else {
    Serial.println("[Register] No ACK after retries - proceeding anyway (ESP-NOW is connectionless).");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(IR_DO_PIN, INPUT);

  // seed debounce state with first real reading
  lastRawReading = digitalRead(IR_DO_PIN);
  stableState = lastRawReading;
  lastChangeTime = millis();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(N4_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.print("[WiFi] Locked to channel: ");
  Serial.println(N4_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = N4_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("N3 (IR) ESP-NOW Sender Ready");
  registerWithGateway();
}

void loop() {
  int raw = digitalRead(IR_DO_PIN);

  // ---- Debounce: only trust a state after it holds steady for DEBOUNCE_MS ----
  if (raw != lastRawReading) {
    lastChangeTime = millis();
    lastRawReading = raw;
  }
  if ((millis() - lastChangeTime) > DEBOUNCE_MS) {
    stableState = lastRawReading;
  }

  // active-low: LOW = obstacle. Send a single clean flag: 1 = obstacle, 0 = clear
  outData.val1 = (stableState == LOW) ? 1 : 0;
  outData.val2 = 0;  // reserved / unused for this sensor
  outData.val3 = 0;  // reserved / unused for this sensor
  outData.seq_num = seqCounter++;

  sendPacket(MSG_SENSOR_DATA);

  Serial.print("Sent -> Obstacle: "); Serial.print(outData.val1 == 1 ? "YES" : "NO");
  Serial.print(" | Seq: "); Serial.print(outData.seq_num);
  Serial.print(" | PrevAckDelay: "); Serial.println(pendingAckDelay);

  delay(500);
}