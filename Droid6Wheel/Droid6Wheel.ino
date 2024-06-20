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
#include <Adafruit_NeoPixel.h>
#include "ESP32MotorControl.h"

ESP32MotorControl motors1_3;


// connect motor controller pins to Arduino digital pins
// motor one
int enA = 32;
int in1 = 33;
int in2 = 25;
// motor two
int enB = 14;
int in3 = 26;
int in4 = 27;

int MotDir[1];
int MotSpeed[1];


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

  if (abs(myData.x) < 10)
    myData.x = 0;
    
  if (abs(myData.y) < 10)
    myData.y = 0;
    

  int16_t left = myData.y + myData.x;
  int16_t right = myData.y - myData.x;
  Serial.print("Left: ");
  Serial.println(left);
  Serial.print("Right: ");
  Serial.println(right);

  int dir = map(myData.x,-255,255,-100,100);
  int thr = map(myData.y,-255,255,-100,100);

  int m1 = thr - dir;
  int m2 = thr + dir;

    m1 = min(100,m1);
    m1 = max(-100,m1);

    m2 = min(100,m2);
    m2 = max(-100,m2);

    motors1_3.setMotor_(0, m1);

    motors1_3.setMotor_(1, m2);

  Serial.println();

}

void StopAll()
{
  // now turn off motors
  motors1_3.setMotor_(0, 0);
  motors1_3.setMotor_(1, 0);
  
  
  MotDir[0] = 0;
  MotDir[1] = 0;

  MotSpeed[0] = 0;
  MotSpeed[1] = 0;  
}



void setup() {
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  motors1_3.attachMotors(in1,in2,enA,in3,in4,enB,0,0,0,20000);
  motors1_3.motorsBreakSet(false,false,false);

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
