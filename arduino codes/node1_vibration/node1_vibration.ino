// ==================== NODE 1: VIBRATION SENSOR (MPU6050) ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Sensor: MPU6050-family accelerometer (GY-521 or clone) via I2C, read directly
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

// ==================== SENSOR CONFIG ====================
// MPU6050 / MPU6500 / MPU9250 (GY-521 and clones) read directly over I2C.
// No Adafruit library: clone chips report a different WHO_AM_I and are
// rejected by Adafruit_MPU6050::begin(), which left N1 with no data.
#define SDA_PIN 21
#define SCL_PIN 22
#define SENSOR_READ_INTERVAL_MS 250
#define SENSOR_RETRY_INTERVAL_MS 2000
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75
#define MPU_LSB_PER_G        16384.0f    // ±2 g full scale
#define GRAVITY_MS2          9.80665f

uint8_t mpuAddr = 0;
bool sensorConnected = false;
uint32_t lastSensorRetry = 0;
uint32_t lastSensorWarning = 0;

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
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

const char* mpuName(uint8_t who) {
  switch (who) {
    case 0x68: return "MPU6050";
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    case 0x72: return "MPU6555 / clone";
    case 0x98: return "clone (0x98)";
    default:   return "unknown / clone";
  }
}

bool initSensor(bool verbose) {
  mpuAddr = 0;
  const uint8_t candidates[2] = {0x68, 0x69};
  for (uint8_t k = 0; k < 2; k++) {
    Wire.beginTransmission(candidates[k]);
    if (Wire.endTransmission() == 0) { mpuAddr = candidates[k]; break; }
  }
  if (mpuAddr == 0) {
    if (verbose) {
      Serial.print("[SENSOR] MPU6050 NOT FOUND: no I2C ACK at 0x68/0x69. I2C devices found:");
      int found = 0;
      for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
      }
      Serial.println(found ? "" : " none — check VCC(3V3)/GND/SDA=GPIO21/SCL=GPIO22");
    }
    return false;
  }
  uint8_t who = 0;
  mpuRead(MPU_REG_WHO_AM_I, &who, 1);
  mpuWrite(MPU_REG_PWR_MGMT_1, 0x00);     // wake up (clears sleep bit)
  delay(10);
  mpuWrite(MPU_REG_ACCEL_CONFIG, 0x00);   // ±2 g
  mpuWrite(MPU_REG_CONFIG, 0x04);         // DLPF ≈ 21 Hz
  Serial.printf("[SENSOR] Accelerometer ready: I2C 0x%02X, WHO_AM_I=0x%02X (%s), SDA=GPIO21 SCL=GPIO22, ±2 g\n",
                mpuAddr, who, mpuName(who));
  return true;
}

// Non-blocking read: a missing sensor is re-probed every 2 s without stalling
// the loop (a blocking retry would make the node miss its TDMA slot).
bool readSensor() {
  if (!sensorConnected) {
    if (millis() - lastSensorWarning >= 5000) {
      lastSensorWarning = millis();
      Serial.println("[SENSOR] WARNING: no accelerometer — N1 sends NO vibration data until it is fixed");
    }
    if (millis() - lastSensorRetry < SENSOR_RETRY_INTERVAL_MS) return false;
    lastSensorRetry = millis();
    sensorConnected = initSensor(false);
    if (sensorConnected) Serial.println("[SENSOR] Accelerometer reconnected!");
    return false;
  }
  uint8_t b[6];
  if (!mpuRead(MPU_REG_ACCEL_XOUT_H, b, 6)) {
    Serial.println("[SENSOR] Accelerometer read failed (I2C) — disconnected?");
    sensorConnected = false;
    return false;
  }
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  accelX = ax / MPU_LSB_PER_G * GRAVITY_MS2;
  accelY = ay / MPU_LSB_PER_G * GRAVITY_MS2;
  accelZ = az / MPU_LSB_PER_G * GRAVITY_MS2;
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
  Serial.printf("Firmware: %s\n", WSN_FW_VERSION);

  // 1. Sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  sensorConnected = initSensor(true);

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
  ble_serial_poll();   // Serial Monitor: status | ble | cal <id> <m> | setn <n>

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
  if (mesh_take_data_opportunity(lastOpportunity)) {
    if (mesh_own_tx_busy(OWN_TX_DATA)) {
      // previous packet still pending → it is retransmitted below
    } else if (hasNewSensorData && mesh.next_hop_to_sink != 0) {
      mesh_own_tx_load(OWN_TX_DATA, mesh_build_sensor_packet(accelX, accelY, accelZ, "vibration"));
      hasNewSensorData = false;
      LOGE("[TDMA] %s: TX opportunity — sending vibration reading seq=%d via N%d\n",
           epochStr(tdma_current_epoch()), mesh.own_tx[OWN_TX_DATA].pkt.seq_num,
           mesh.next_hop_to_sink);
    } else if (mesh.tdma.synced) {
      mesh_note_unused_slot(hasNewSensorData ? "no route to sink" : "no new sensor sample");
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
