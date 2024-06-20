/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0xE9, 0xA8, 0x70};

// R2 uint8_t broadcastAddress[] = {0x84, 0xCC, 0xA8, 0x69, 0xB2, 0x94};
// Imperial BT1 uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xE5, 0x49, 0x60};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  uint16_t Header;
  int16_t x;
  int16_t y;
  bool btn;
  uint8_t touch1;
  uint8_t touch2;
  uint16_t Footer;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(34, INPUT_PULLDOWN);

}
 
void loop() {
  // Set values to send
  myData.y = -(map(analogRead(32),0,4096,256,-256) - 7) ;
  myData.x = (map(analogRead(33),0,4096,256,-256) - 12);
  myData.btn = digitalRead(34);

  myData.Header = 0xAAAF;
  myData.Footer = 0xAAAA;

  Serial.print(myData.x); Serial.print(' ');Serial.println(myData.y); 
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  //Serial.print(millis());Serial.print(" ");
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(50);
}
