#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include "SPIFFS.h"

typedef struct struct_message {
  float readings[10]; // Adjust based on your needs
} struct_message;

struct_message myData;
uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x55, 0xE4, 0x2C};
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
}

void readFileAndSend() {
    Serial.println("Opening file...");
    File file = SPIFFS.open("/data_file.txt", "r");
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    } else {
        Serial.println("File opened successfully");
    }

    struct_message dataToSend;
    int index = 0;

    while (file.available() && index < 10) {
        Serial.print("Reading line ");
        Serial.println(index + 1);
        String line = file.readStringUntil('\n');
        if (line.length() > 0) { // Check if line is not empty
            float value = line.toFloat();
            dataToSend.readings[index] = value;
            Serial.print("Value: ");
            Serial.println(dataToSend.readings[index]);
            index++;
        } else {
            Serial.println("Empty line or conversion failed");
        }
    }
    file.close();

    if (index == 0) {
        Serial.println("No data read from file.");
        return; // Exit if no data was read
    }

    // Now, send dataToSend using ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
    if (result == ESP_OK) {
        Serial.println("Data sent with success");
    } else {
        Serial.println("Error sending data");
    }
}



void loop() {
  readFileAndSend(); // Read from file and send
  delay(10000); // Delay before next send
}
