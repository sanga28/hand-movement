#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL3oQ_kg68I"
#define BLYNK_TEMPLATE_NAME "handtracking1"
#define BLYNK_AUTH_TOKEN "uJQw9qZI2Nb5if00zm1NkCqZVZPyK4nU"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>

char ssid[] = "IEDC LAB 505";
char pass[] = "Iedclab505";

// Servo pin definitions
#define pin1 12  // Index
#define pin2 13  // Middle
#define pin3 14  // Ring
#define pin4 15  // Little
#define pin5 5   // Thumb
#define pin6 4   // Wrist Vertical
#define pin7 2   // Wrist Horizontal
#define pin8 0   // Thumb Side

Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8;

int index_angle = 0, middle_angle = 0, ring_angle = 0, little_angle = 0;
int thumb_angle = 0, thumb_side_angle = 0, wrist_angle = 0, wrist_side_angle = 0;

void setup() {
  Serial.begin(115200);

  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);
  servo4.attach(pin4);
  servo5.attach(pin5);
  servo6.attach(pin6);
  servo7.attach(pin7);
  servo8.attach(pin8);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

// Each BLYNK_WRITE updates angle and prints combined formatted line
BLYNK_WRITE(V0) {
  index_angle = param.asInt();
  servo1.write(index_angle * 20);
  printAllAngles();
}

BLYNK_WRITE(V1) {
  middle_angle = param.asInt();
  servo2.write(middle_angle * 20);
  printAllAngles();
}

BLYNK_WRITE(V2) {
  ring_angle = param.asInt();
  servo3.write(ring_angle * 20);
  printAllAngles();
}

BLYNK_WRITE(V3) {
  little_angle = param.asInt();
  servo4.write(little_angle * 20);
  printAllAngles();
}

BLYNK_WRITE(V4) {
  thumb_angle = param.asInt();
  servo5.write(thumb_angle * 20);
  printAllAngles();
}

BLYNK_WRITE(V5) {
  thumb_side_angle = param.asInt();
  servo8.write(thumb_side_angle * 20);
  printAllAngles();
}

BLYNK_WRITE(V7) {
  wrist_angle = param.asInt();
  servo6.write(wrist_angle * 20);
  printAllAngles();
}

BLYNK_WRITE(V8) {
  wrist_side_angle = param.asInt();
  servo7.write(wrist_side_angle * 20);
  printAllAngles();
}

// Print all joint values to Serial in a single line
void printAllAngles() {
  Serial.print("ind_"); Serial.print(index_angle);
  Serial.print(":mid_"); Serial.print(middle_angle);
  Serial.print(":rin_"); Serial.print(ring_angle);
  Serial.print(":lit_"); Serial.print(little_angle);
  Serial.print(":thu_"); Serial.print(thumb_angle);
  Serial.print(":ths_"); Serial.print(thumb_side_angle);
  Serial.print(":wri_"); Serial.print(wrist_angle);
  Serial.print(":wrh_"); Serial.println(wrist_side_angle);
}

void loop() {
  Blynk.run();
}

