#define BLYNK_PRINT Serial

// Blynk Credentials
#define BLYNK_TEMPLATE_ID "TMPL3ZlseNfCE"
#define BLYNK_TEMPLATE_NAME "handtracking"
#define BLYNK_AUTH_TOKEN "2Ody8bjt9v9XTXmZchbbA5hdxlHi1pEP"

#include <Servo.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// WiFi Credentials
char ssid[] = "IEDC LAB 505";
char pass[] = "Iedclab505";

// Servo Pin Mapping
#define pin1 12  // Index
#define pin2 13  // Middle
#define pin3 14  // Ring
#define pin4 15  // Little
#define pin5 5   // Thumb
#define pin6 4   // Wrist Vertical
#define pin7 2   // Wrist Horizontal (NEW! Change to your free pin)

Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7; // NEW: servo7 for horizontal wrist

int index_angle = 0;
int middle_angle = 0;
int ring_angle = 0;
int little_angle = 0;
int thumb_angle = 0;
int thumb_side_angle = 0;
int wrist_angle = 0;
int wrist_side_angle = 0;  // NEW

void setup() {
  Serial.begin(115200);

  // Attach servos
  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);
  servo4.attach(pin4);
  servo5.attach(pin5);
  servo6.attach(pin6);
  servo7.attach(pin7);  // NEW

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    if (data.startsWith("ind_")) {
      sscanf(data.c_str(), 
        "ind_%d:mid_%d:rin_%d:lit_%d:thu_%d:ths_%d:wri_%d:wrh_%d",  // UPDATED
        &index_angle, &middle_angle, &ring_angle, &little_angle,
        &thumb_angle, &thumb_side_angle, &wrist_angle, &wrist_side_angle);  // NEW

      Serial.print("Received Data: ");
      Serial.println(data);

      // Move servos
      servo1.write(index_angle * 20);
      servo2.write(middle_angle * 20);
      servo3.write(ring_angle * 20);
      servo4.write(little_angle * 20);
      servo5.write(thumb_angle * 20);
      servo6.write(wrist_angle * 20);
      servo7.write(wrist_side_angle * 20);  // NEW

      // Send to Blynk
      Blynk.virtualWrite(V0, index_angle);
      Blynk.virtualWrite(V1, middle_angle);
      Blynk.virtualWrite(V2, ring_angle);
      Blynk.virtualWrite(V3, little_angle);
      Blynk.virtualWrite(V4, thumb_angle);
      Blynk.virtualWrite(V5, thumb_side_angle);
      Blynk.virtualWrite(V6, data);
      Blynk.virtualWrite(V7, wrist_angle);
      Blynk.virtualWrite(V8, wrist_side_angle);  // NEW
    }
  }
}
