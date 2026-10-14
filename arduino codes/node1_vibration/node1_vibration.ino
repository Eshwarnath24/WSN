// ==================== NODE 1: VIBRATION SENSOR (MPU6050) ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Sensor: Adafruit MPU6050 6-axis accelerometer via I2C
// Primary route: N1 → N3 → N4 (aggregated)
// Alternate:     N1 → N4 direct (N3 down) or N1 → N2 → … (N1–N4 link down)
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
#define MPU_ADDR 0x68
#define SENSOR_READ_INTERVAL_MS 250
#define SENSOR_RETRY_INTERVAL_MS 2000

Adafruit_MPU6050 mpu;
bool sensorConnected = false;
uint32_t lastSensorRetry = 0;

// Latest sensor readings
float accelX = 0, accelY = 0, accelZ = 0;
bool hasNewSensorData = false;

// ==================== TIMERS ====================
uint32_t lastSensorRead = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;

// ==================== APP CALLBACKS ====================
void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  Serial.printf("[APP] Data arrived for N1: type=%s origin=N%d seq=%d\n",
    msgTypeStr(pkt.msg_type), pkt.origin_id, pkt.seq_num);
}

// Intermediate hop: N1 has no aggregation role — forward normally
bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  return false;
}

void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {
  // Collision test is handled by mesh_collision_test_service()
}

// ==================== SENSOR FUNCTIONS ====================
void configureMpu() {
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

bool initSensor() {
  Wire.begin(SDA_PIN, SCL_PIN);
  if (mpu.begin(MPU_ADDR)) {
    configureMpu();
    Serial.println("[SENSOR] MPU6050 initialized (I2C 0x68, SDA=GPIO21, SCL=GPIO22)");
    return true;
  }
  Serial.println("[SENSOR] MPU6050 NOT FOUND (check VCC/GND/SDA/SCL and AD0=GND)");
  return false;
}

// Non-blocking read: a missing sensor is re-probed every 2 s without stalling
// the loop (a blocking retry would make the node miss its TDMA slot).
bool readSensor() {
  Wire.beginTransmission(MPU_ADDR);
  byte err = Wire.endTransmission();
  if (err != 0) {
    if (sensorConnected) {
      Serial.printf("[SENSOR] MPU6050 disconnected (I2C err=%d) — no readings will be sent\n", err);
    }
    sensorConnected = false;
    return false;
  }
  if (!sensorConnected) {
    if (millis() - lastSensorRetry < SENSOR_RETRY_INTERVAL_MS) return false;
    lastSensorRetry = millis();
    if (!mpu.begin(MPU_ADDR)) return false;
    configureMpu();
    sensorConnected = true;
    Serial.println("[SENSOR] MPU6050 reconnected!");
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;
  return true;
}

float vibrationMagnitude() {
  return sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
}

// ==================== SETUP ====================
void setup() {
  Serial.setTxBufferSize(4096);   // keep Serial output from blocking the radio callbacks
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 1 (Vibration Sensor) BOOTING =====");

  // 1. Sensor
  sensorConnected = initSensor();

  // 2. BLE advertising + scanning, boot-time discovery / RSSI / localization
  ble_init();
  ble_boot_discovery();

  // 3. Wi-Fi / ESP-NOW mesh stack (MAC, routing, TDMA)
  mesh_init();

  Serial.println("===== N1 SETUP COMPLETE =====\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t now = millis();

  // 1. Mesh periodic tasks (sync, HELLO, schedule, forwarding)
  mesh_loop();

  // 2. Contention demo (waits for CAP start)
  mesh_collision_test_service();

  // 3. BLE scan burst in SLEEP epoch + neighbour expiry
  ble_tick();

  // 4. BLE report to sink every 10 s (sent in the CAP, multi-hop)
  if (now - lastBleReport >= BLE_REPORT_PERIOD_MS && !mesh_own_tx_busy(OWN_TX_BLE)) {
    lastBleReport = now;
    ble_report_payload_t rpt = ble_build_report();
    if (rpt.num_neighbors > 0) {
      mesh_own_tx_load(OWN_TX_BLE, mesh_build_ble_report_packet(rpt));
    }
  }
  if (mesh_own_tx_busy(OWN_TX_BLE) && mesh_ctrl_window_open()) {
    mesh_own_tx_service(OWN_TX_BLE);
  }

  // 5. Read sensor at fixed interval (buffer latest sample)
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;
    if (readSensor()) {
      hasNewSensorData = true;
      static uint8_t printDiv = 0;
      if (++printDiv >= 4) {   // print once per second
        printDiv = 0;
        Serial.printf("[SENSOR] AX=%.2f AY=%.2f AZ=%.2f mag=%.2f m/s^2%s\n",
                      accelX, accelY, accelZ, vibrationMagnitude(),
                      vibrationMagnitude() > VIBRATION_ALERT_THRESHOLD ? " (ANOMALY)" : "");
      }
    }
  }

  // 6. TDMA: one new packet per assigned TX slot occurrence
  static uint32_t lastOpportunity = 0xFFFFFFFF;
  if (mesh_data_window_open()) {
    uint32_t key = mesh_tx_opportunity_key();
    if (key != lastOpportunity) {
      lastOpportunity = key;
      if (mesh_own_tx_busy(OWN_TX_DATA)) {
        // previous packet still pending → it is retransmitted below
      } else if (hasNewSensorData && mesh.next_hop_to_sink != 0) {
        mesh_own_tx_load(OWN_TX_DATA, mesh_build_sensor_packet(accelX, accelY, accelZ, "vibration"));
        hasNewSensorData = false;
        Serial.printf("[TDMA] %s: TX opportunity — sending vibration reading seq=%d via N%d\n",
                      epochStr(tdma_current_epoch()), mesh.own_tx[OWN_TX_DATA].pkt.seq_num,
                      mesh.next_hop_to_sink);
      } else if (mesh.tdma.synced) {
        mesh_note_unused_slot(hasNewSensorData ? "no route to sink" : "no new sensor sample");
      }
    }
  }
  // (Re)transmit pending own data in own slot or CAP
  if (mesh_own_tx_busy(OWN_TX_DATA) && mesh_retry_window_open()) {
    mesh_own_tx_service(OWN_TX_DATA);
  }

  // 7. Print status periodically
  if (now - lastStatusPrint >= 10000) {
    lastStatusPrint = now;
    mesh_print_status();
    ble_print_status();
  }
}
