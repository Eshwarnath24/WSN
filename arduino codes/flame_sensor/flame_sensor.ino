#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

uint8_t receiverMAC[] = {0x8C, 0x94, 0xDF, 0x7C, 0x3F, 0x58};
#define N4_CHANNEL 11  // must match N4's actual printed channel

#define FLAME_AO_PIN 32
#define FLAME_DO_PIN 16

typedef struct struct_message {
  char node_id[4];
  char sensor_type[10];
  float val1;
  float val2;
  float val3;
  int seq_num;
} struct_message;

struct_message myData;
int seqCounter = 0;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  pinMode(FLAME_DO_PIN, INPUT);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(N4_CHANNEL, WIFI_SECOND_CHAN_NONE);

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

  Serial.println("N2 (Flame) ESP-NOW Sender Ready");
}

void loop() {
  strcpy(myData.node_id, "N2");
  strcpy(myData.sensor_type, "flame");
  myData.val1 = analogRead(FLAME_AO_PIN);   // analog flame reading
  myData.val2 = digitalRead(FLAME_DO_PIN);  // digital flame state
  myData.val3 = 0;                          // unused for this sensor
  myData.seq_num = seqCounter++;

  esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));

  Serial.print("Sent -> Analog: ");
  Serial.print(myData.val1);
  Serial.print(" | Digital: ");
  Serial.print(myData.val2);
  Serial.print(" | Seq: ");
  Serial.println(myData.seq_num);

  delay(1000);
}