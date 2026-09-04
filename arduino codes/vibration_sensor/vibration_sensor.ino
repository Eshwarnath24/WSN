// ================= NODE 1 (Vibration Sensor) - SENDER =================
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

uint8_t receiverMAC[] = {0x8C, 0x94, 0xDF, 0x7C, 0x3F, 0x58}; // N4 gateway MAC
#define N4_CHANNEL 11  // must match N4's printed channel

#define SDA_PIN 21
#define SCL_PIN 22

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
  unsigned long ack_delay_ms;   // send->ack delay measured for the PREVIOUS packet (0 until known)
} struct_message;

struct_message outData;

Adafruit_MPU6050 mpu;
bool sensorConnected = false;
unsigned long seqCounter = 0;

volatile bool registered = false;
volatile bool ackReceivedFlag = false;
unsigned long lastSendMillis = 0;
volatile unsigned long pendingAckDelay = 0;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  unsigned long now = millis();
  if (status == ESP_NOW_SEND_SUCCESS) {
    pendingAckDelay = now - lastSendMillis;
  }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  struct_message temp;
  memcpy(&temp, data, sizeof(temp));
  if (temp.msg_type == MSG_REGISTER_ACK && String(temp.node_id) == "N1") {
    registered = true;
    ackReceivedFlag = true;
    Serial.println("[Register] REGISTER_ACK received from N4");
  }
}

void sendPacket(uint8_t type) {
  outData.msg_type = type;
  strcpy(outData.node_id, "N1");
  strcpy(outData.sensor_type, "vibration");
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
    Serial.println("[Register] N1 REGISTERED with gateway.");
  } else {
    Serial.println("[Register] No ACK after retries - proceeding anyway (ESP-NOW is connectionless).");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Starting MPU6050...");
  sensorConnected = mpu.begin();
  if (sensorConnected) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 detected!");
  } else {
    Serial.println("MPU6050 NOT FOUND! Will keep retrying...");
  }

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

  Serial.println("N1 ESP-NOW Sender Ready");
  registerWithGateway();
}

void loop() {
  Wire.beginTransmission(0x68);
  byte error = Wire.endTransmission();

  if (error != 0) {
    sensorConnected = false;
    Serial.println("MPU6050 DISCONNECTED! Retrying...");
    delay(500);
    sensorConnected = mpu.begin();
    if (sensorConnected) {
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.println("MPU6050 RECONNECTED!");
    }
    return;
  }

  sensorConnected = true;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  outData.val1 = accel.acceleration.x;
  outData.val2 = accel.acceleration.y;
  outData.val3 = accel.acceleration.z;
  outData.seq_num = seqCounter++;

  sendPacket(MSG_SENSOR_DATA);

  Serial.print("Sent -> AX: "); Serial.print(outData.val1);
  Serial.print(" | AY: "); Serial.print(outData.val2);
  Serial.print(" | AZ: "); Serial.print(outData.val3);
  Serial.print(" | Seq: "); Serial.print(outData.seq_num);
  Serial.print(" | PrevAckDelay: "); Serial.println(pendingAckDelay);

  delay(500);
}