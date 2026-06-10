#include <WiFi.h>
#include <esp_now.h>

// EDIT THESE PINS IF NEEDED
#define F9R_RX_PIN 16   // ESP32 RX, unused for corrections
#define F9R_TX_PIN 17   // ESP32 TX connected to F9R RX2

#define RTCM_BAUD 115200

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Serial2.write(data, len);

  Serial.print("Wrote RTCM bytes to F9R: ");
  Serial.println(len);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Rover ESP32 RTCM bridge starting...");

  Serial2.begin(RTCM_BAUD, SERIAL_8N1, F9R_RX_PIN, F9R_TX_PIN);

  WiFi.mode(WIFI_STA);
  delay(500);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  Serial.println("Rover ESP32 ready");
}

void loop() {}
