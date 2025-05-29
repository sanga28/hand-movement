#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL3oQ_kg68I"
#define BLYNK_TEMPLATE_NAME "handtracking1"
#define BLYNK_AUTH_TOKEN "uJQw9qZI2Nb5if00zm1NkCqZVZPyK4nU"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char ssid[] = "IEDC LAB 505";
char pass[] = "Iedclab505";

// Joint angles
int index_angle = 0, middle_angle = 0, ring_angle = 0, little_angle = 0;
int thumb_angle = 0, thumb_side_angle = 0, wrist_angle = 0, wrist_side_angle = 0;

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    if (data.startsWith("ind_")) {
      sscanf(data.c_str(), 
        "ind_%d:mid_%d:rin_%d:lit_%d:thu_%d:ths_%d:wri_%d:wrh_%d",
        &index_angle, &middle_angle, &ring_angle, &little_angle,
        &thumb_angle, &thumb_side_angle, &wrist_angle, &wrist_side_angle);

      // Upload to Blynk
      Blynk.virtualWrite(V0, index_angle);
      Blynk.virtualWrite(V1, middle_angle);
      Blynk.virtualWrite(V2, ring_angle);
      Blynk.virtualWrite(V3, little_angle);
      Blynk.virtualWrite(V4, thumb_angle);
      Blynk.virtualWrite(V5, thumb_side_angle);
      Blynk.virtualWrite(V6, wrist_angle);
      Blynk.virtualWrite(V7, wrist_side_angle);

      Serial.println("Data sent to Blynk: " + data);
    }
  }
}





// "ind_180:mid_180:rin_180:lit_180:thu_180:ths_180"




