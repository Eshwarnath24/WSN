// ================= NODE 1 (Vibration Sensor) - SENDER =================
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

uint8_t receiverMAC[] = {0x8C, 0x94, 0xDF, 0x7C, 0x3F, 0x58}; // N4 gateway MAC
#define MIN_CHANNEL 1
#define MAX_CHANNEL 13
#define CHANNEL_ACK_WAIT_MS 300
#define RESCAN_FAIL_THRESHOLD 15   // after this many consecutive send failures, re-scan channels

int lockedChannel = -1;
volatile int consecutiveFails = 0;

#define SDA_PIN 21
#define SCL_PIN 22
#define TELEMETRY_INTERVAL_MS 250 // Reduced delay for faster real-time telemetry (was 500ms)

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
    consecutiveFails = 0;
  } else {
    consecutiveFails++;
  }
  Serial.print("[ESP-NOW] Send status: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
  Serial.print(" | Channel: ");
  Serial.println(lockedChannel);
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

void findGatewayAndRegister() {
  registered = false;
  int sweep = 0;

  while (!registered) {
    sweep++;
    Serial.print("[Channel Scan] Sweep #"); Serial.println(sweep);

    for (int ch = MIN_CHANNEL; ch <= MAX_CHANNEL && !registered; ch++) {
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      ackReceivedFlag = false;
      sendPacket(MSG_REGISTER);

      unsigned long waitStart = millis();
      while (!ackReceivedFlag && millis() - waitStart < CHANNEL_ACK_WAIT_MS) {
        delay(5);
      }

      if (ackReceivedFlag) {
        registered = true;
        lockedChannel = ch;
        consecutiveFails = 0;
        Serial.print("[Channel Scan] Gateway found on channel ");
        Serial.println(ch);
      }
    }

    if (!registered) {
      Serial.println("[Channel Scan] Gateway not found this sweep. Retrying in 1s...");
      delay(1000);
    }
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
  // Channel will be set dynamically by findGatewayAndRegister()

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;   // use whatever channel is currently set
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("N1 ESP-NOW Sender Ready");
  findGatewayAndRegister();
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

  if (consecutiveFails >= RESCAN_FAIL_THRESHOLD) {
    Serial.println("[Channel Scan] Too many failures - gateway may have changed channel. Rescanning...");
    findGatewayAndRegister();
  }

  Serial.print("Sent -> AX: "); Serial.print(outData.val1);
  Serial.print(" | AY: "); Serial.print(outData.val2);
  Serial.print(" | AZ: "); Serial.print(outData.val3);
  Serial.print(" | Seq: "); Serial.print(outData.seq_num);
  Serial.print(" | PrevAckDelay: "); Serial.println(pendingAckDelay);

  delay(TELEMETRY_INTERVAL_MS);
}