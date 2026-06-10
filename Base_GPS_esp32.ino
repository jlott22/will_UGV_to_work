#include <WiFi.h>
#include <esp_now.h>

// Rover ESP32 MAC: 68:FE:71:0B:76:D0
uint8_t roverMac[] = {0x68, 0xFE, 0x71, 0x0B, 0x76, 0xD0};

// EDIT THESE PINS IF NEEDED
#define F9P_RX_PIN 16   // ESP32 RX pin connected to F9P TX2
#define F9P_TX_PIN 17   // unused for now

#define RTCM_BAUD 115200
#define MAX_PACKET_SIZE 200

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "success" : "fail");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Base ESP32 RTCM bridge starting...");

  Serial2.begin(RTCM_BAUD, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);

  WiFi.mode(WIFI_STA);
  delay(500);

  Serial.print("Base STA MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.printf(
    "Target rover MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    roverMac[0], roverMac[1], roverMac[2],
    roverMac[3], roverMac[4], roverMac[5]
  );

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  Serial.println("ESP-NOW initialized");

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, roverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t addStatus = esp_now_add_peer(&peerInfo);

  if (addStatus == ESP_OK) {
    Serial.println("Rover peer added successfully");
  } else {
    Serial.print("Failed to add rover peer. Error code: ");
    Serial.println(addStatus);
    return;
  }

  Serial.println("Base ESP32 ready");
}

void loop() {
  uint8_t buffer[MAX_PACKET_SIZE];

  int n = 0;
  while (Serial2.available() && n < MAX_PACKET_SIZE) {
    buffer[n++] = Serial2.read();
  }

  if (n > 0) {
    esp_err_t result = esp_now_send(roverMac, buffer, n);

    Serial.print("Forwarded RTCM bytes: ");
    Serial.println(n);

    if (result == ESP_OK) {
      Serial.println("esp_now_send call accepted");
    } else {
      Serial.print("esp_now_send call failed immediately. Error code: ");
      Serial.println(result);
    }
  }
}
