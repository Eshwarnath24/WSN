// ==================== NODE 1: VIBRATION SENSOR (MPU6050) ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Sensor: Adafruit MPU6050 6-axis accelerometer via I2C
// Primary route: N1 → N3 → N4    Alternate: N1 → N2 → N4
// ============================================================================

#define MY_NODE_ID 1
#if __has_include("wsn_common.h")
#include "wsn_common.h"
#include "wsn_mesh.h"
#include "wsn_ble.h"
#else
#include "../common/wsn_common.h"
#include "../common/wsn_mesh.h"
#include "../common/wsn_ble.h"
#endif

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ==================== SENSOR CONFIG ====================
#define SDA_PIN 21
#define SCL_PIN 22
#define SENSOR_READ_INTERVAL_MS 250

Adafruit_MPU6050 mpu;
bool sensorConnected = false;

// Latest sensor readings
float accelX = 0, accelY = 0, accelZ = 0;
bool hasNewSensorData = false;

// Vibration anomaly threshold (for dynamic TDMA demo)
#ifndef VIBRATION_ALERT_THRESHOLD
#define VIBRATION_ALERT_THRESHOLD 15.0f  // m/s^2
#endif

// ==================== TIMERS ====================
uint32_t lastSensorRead = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;

// ==================== APP CALLBACKS ====================
// Called when a packet addressed to us (final_dst=N1) arrives.
// N1 is a sensor, not the sink, so this is rare (e.g., control msgs).
void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  Serial.printf("[APP] Data arrived for N1: type=%s origin=N%d seq=%d\n",
    msgTypeStr(pkt.msg_type), pkt.origin_id, pkt.seq_num);
}

// Called when data transits through us (intermediate hop). Return true to consume.
bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  return false;  // N1 has no aggregation role — forward normally
}

// Called on CTRL commands
void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {
  // Collision test is handled automatically by mesh_collision_test_send()
}

// ==================== SENSOR FUNCTIONS ====================
bool initSensor() {
  Wire.begin(SDA_PIN, SCL_PIN);
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[SENSOR] MPU6050 initialized");
    return true;
  }
  Serial.println("[SENSOR] MPU6050 NOT FOUND");
  return false;
}

bool readSensor() {
  // Check I2C connection
  Wire.beginTransmission(0x68);
  byte err = Wire.endTransmission();
  if (err != 0) {
    sensorConnected = false;
    Serial.println("[SENSOR] MPU6050 disconnected, retrying...");
    delay(200);
    sensorConnected = mpu.begin();
    if (sensorConnected) {
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.println("[SENSOR] MPU6050 reconnected!");
    }
    return false;
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;
  sensorConnected = true;
  return true;
}

float vibrationMagnitude() {
  return sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 1 (Vibration Sensor) BOOTING =====");

  // 1. Initialize sensor
  sensorConnected = initSensor();

  // 2. Initialize BLE (advertising + scanning)
  ble_init();

  // 3. Initialize mesh stack (ESP-NOW, MAC, routing, TDMA)
  mesh_init();

  Serial.println("===== N1 SETUP COMPLETE =====\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t now = millis();

  // 1. Run mesh periodic tasks (HELLO, timeouts, etc.)
  mesh_loop();

  // 2. Handle collision test trigger
  mesh_collision_test_send();

  // 3. BLE scan during sleep epoch
  tdma_epoch_t epoch = tdma_current_epoch();
  if (epoch == EPOCH_SLEEP && now - mesh.last_ble_scan_ms >= 3000) {
    ble_scan();
    mesh.last_ble_scan_ms = now;
  }

  // 4. Send BLE report to sink periodically (every 10s)
  if (now - lastBleReport >= 10000) {
    lastBleReport = now;
    ble_report_payload_t rpt = ble_build_report();
    if (rpt.num_neighbors > 0) {
      mesh_send_ble_report(rpt);
    }
  }

  // 5. Read sensor at fixed interval (buffer latest sample)
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;
    if (readSensor()) {
      hasNewSensorData = true;
      Serial.printf("[SENSOR] AX=%.2f AY=%.2f AZ=%.2f mag=%.2f\n",
                    accelX, accelY, accelZ, vibrationMagnitude());
    }
  }

  // 6. Transmit sensor data strictly during assigned TDMA TX slot (Defect 3 fix)
  static tdma_epoch_t last_tx_slot = EPOCH_SLEEP;
  static uint32_t last_tx_frame = 0xFFFFFFFF;
  uint32_t current_frame = mesh.tdma.frame_number;
  tdma_epoch_t curr_epoch = tdma_current_epoch();

  if (mesh.tdma.synced) {
    // Only transmit when in our assigned TX slot, once per slot occurrence
    if (tdma_is_my_tx_slot()) {
      if (curr_epoch != last_tx_slot || current_frame != last_tx_frame) {
        if (hasNewSensorData && mesh.next_hop_to_sink != 0) {
          mesh_send_sensor_data(accelX, accelY, accelZ, "vibration");
          hasNewSensorData = false;
          last_tx_slot = curr_epoch;
          last_tx_frame = current_frame;
        }
      }
    } else {
      last_tx_slot = curr_epoch;
    }
  } else {
    // Free-running fallback when unsynced: transmit every 1s
    static uint32_t lastUnsyncedTx = 0;
    if (now - lastUnsyncedTx >= 1000) {
      lastUnsyncedTx = now;
      if (hasNewSensorData && mesh.next_hop_to_sink != 0) {
        mesh_send_sensor_data(accelX, accelY, accelZ, "vibration");
        hasNewSensorData = false;
      }
    }
  }

  // 7. Print status periodically
  if (now - lastStatusPrint >= 10000) {
    lastStatusPrint = now;
    mesh_print_status();
    ble_print_status();
  }
}