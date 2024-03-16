#include <esp_now.h>
#include <WiFi.h>

#define MotorR1 32 //GPIO32
#define MotorR2 33 //GPIO33 
#define MotorRE 25 //GPIO25 

#define MotorL1 35 //GPIO26   
#define MotorL2 34 //GPIO27  
#define MotorLE 26 //GPIO14 


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int turn[2];
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Left: ");
  Serial.println(myData.turn[0]);
  Serial.print("Rigth: "); 
  Serial.println(myData.turn[1]); 


  digitalWrite(MotorR1, HIGH);
  digitalWrite(MotorR2, LOW);
  analogWrite(MotorRE, 100);

  digitalWrite(MotorL1, HIGH);
  digitalWrite(MotorL2, LOW);
  analogWrite(MotorLE, 100);

}
 
void setup() {
  
  pinMode(MotorR1, OUTPUT);
  pinMode(MotorR2, OUTPUT);
  pinMode(MotorL1, OUTPUT);
  pinMode(MotorL2, OUTPUT);
  pinMode(MotorRE, OUTPUT);
  pinMode(MotorLE, OUTPUT);

  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  

}
 
void loop() {



}