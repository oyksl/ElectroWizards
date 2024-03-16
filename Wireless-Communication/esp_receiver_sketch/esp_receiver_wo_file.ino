#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  float readings[100]; // Ensure this matches the sender structure
} __attribute__((packed)) struct_message;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    Serial.print("Received data size: ");
    Serial.println(len);
    Serial.println("Received data values:");

    // Attempt to print data even if it's incomplete, for debugging purposes
    int numFloatsReceived = len / sizeof(float); // Calculate how many floats were received
    for (int i = 0; i < numFloatsReceived; i++) {
        // Copy each received float into a temporary variable to print
        float temp;
        memcpy(&temp, &incomingData[i * sizeof(float)], sizeof(float));
        Serial.println(temp, 6); // Print each float with 6 digits after decimal
    }

    // Check if the received data length matches the expected size of our structure
    if (len == sizeof(struct_message)) {
        struct_message myData;
        memcpy(&myData, incomingData, sizeof(myData));
        Serial.println("Data is complete. Processing it...");
        // Optionally, process the data here
    } else {
        Serial.println("Warning: Received incomplete data.");
    }
}


void setup() {
  Serial.begin(115200); // Start Serial Monitor
  WiFi.mode(WIFI_STA); // Set device to STA mode

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing to do here
}
