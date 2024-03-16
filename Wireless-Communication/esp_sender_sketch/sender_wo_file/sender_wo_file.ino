#include <esp_now.h>
#include <WiFi.h>

// Define a struct for sending data
typedef struct struct_message {
    float readings[100]; // Adjust the size according to your needs
} struct_message;

struct_message myData;
uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x55, 0xE4, 0x2C}; // Replace with your receiver's MAC address

// Callback when data send is complete
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Data send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo)); // Clear the peer information structure.
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0; 
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    static bool dataReadyToSend = false; // Flag to track when we have a complete set of data to send

    if (Serial.available()) {
        String incomingData = Serial.readStringUntil('\n'); // Read the incoming data as a string

        // Parse incoming data to float array (assuming CSV format)
        int index = 0;
        char* ptr = strtok((char*)incomingData.c_str(), ",");
        while (ptr != NULL && index < sizeof(myData.readings) / sizeof(myData.readings[0])) {
            myData.readings[index++] = atof(ptr);
            ptr = strtok(NULL, ",");
        }

        if (index > 0) { // Check if we have at least one reading
            dataReadyToSend = true; // We have a complete set of data ready to send
        }
    }

    // Check if it's time to send the data
    if (dataReadyToSend) {
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
        if (result == ESP_OK) {
            Serial.println("Data sent with success");
        } else {
            Serial.println("Error sending data");
        }
        dataReadyToSend = false; // Reset the flag until new data is ready
    }
}


