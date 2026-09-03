// ================= NODE 1 (Vibration Sensor) - SENDER =================
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

uint8_t receiverMAC[] = {0x8C, 0x94, 0xDF, 0x7C, 0x3F, 0x58}; // N4 gateway MAC
#define N4_CHANNEL 11  // <-- REPLACE with the channel number N4 prints

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_MPU6050 mpu;
bool sensorConnected = false;
int seqCounter = 0;

typedef struct struct_message {
  char node_id[4];
  char sensor_type[10];
  float val1;
  float val2;
  float val3;
  int seq_num;
} struct_message;

struct_message myData;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
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

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = N4_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("N1 ESP-NOW Sender Ready");
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

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  strcpy(myData.node_id, "N1");
  strcpy(myData.sensor_type, "vibration");
  myData.val1 = ax;
  myData.val2 = ay;
  myData.val3 = az;
  myData.seq_num = seqCounter++;

  esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));

  Serial.print("Sent -> AX: "); Serial.print(ax);
  Serial.print(" | AY: "); Serial.print(ay);
  Serial.print(" | AZ: "); Serial.print(az);
  Serial.print(" | Seq: "); Serial.println(myData.seq_num);

  delay(500);
}