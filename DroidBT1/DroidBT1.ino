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
#include <ESP32Servo.h> 
#include <Adafruit_NeoPixel.h>


Servo leftServo;  // create servo object to control a servo
Servo rightServo;  // create servo object to control a servo
Adafruit_NeoPixel strip(10, 13, NEO_GRB + NEO_KHZ800);

// Structure example to receive data
// Must match the sender structure
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

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Header: ");
  Serial.println(myData.Header);
  Serial.print("Footer: ");
  Serial.println(myData.Footer);
  Serial.print("X: ");
  Serial.println(myData.x);
  Serial.print("Y: ");
  Serial.println(myData.y);
  Serial.print("Btn: ");
  Serial.println(myData.btn);
  Serial.print("Touch1: ");
  Serial.println(myData.touch1);
  Serial.print("Touch2: ");
  Serial.println(myData.touch2);

  if (myData.Header != 0xAAAF)
    return;

  if (myData.Footer != 0xAAAA)
    return;

  int16_t left = (-myData.y) + myData.x;
  int16_t right = (-myData.y) - myData.x;
  Serial.print("Left: ");
  Serial.println(left);
  Serial.print("Right: ");
  Serial.println(right);

  leftServo.write(map(left,-300,300,1000,2000));                  // set the servo position according to the scaled value
  rightServo.write(map(right,-300,300,1000,2000));                  // set the servo position according to the scaled value


  Serial.println();

}

 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.println(WiFi.macAddress());


  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

    ESP32PWM::allocateTimer(0);
    leftServo.setPeriodHertz(50);// Standard 50hz servo
    leftServo.attach(14, 500, 2400);   // attaches the servo on pin 12
    leftServo.write(1500);                  // set the servo position according to the scaled value

    rightServo.setPeriodHertz(50);// Standard 50hz servo
    rightServo.attach(12, 500, 2400);   // attaches the servo on pin 12
    rightServo.write(1500);                  // set the servo position according to the scaled value

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

}
 
void loop() {
  for (int i =0; i < 12; i++)
  {
      strip.setPixelColor(i, strip.Color(  random(255),   random(255), random(255)));
  }
  strip.show();            // Turn OFF all pixels ASAP
  delay(100);
  

}
