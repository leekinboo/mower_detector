#include <Arduino.h>
#include <HardwareSerial.h>
#include "UNIT_UHF_RFID.h"

Unit_UHF_RFID RFID(&Serial);
bool DEBUG = true;


void sendCMD(uint8_t *data, size_t size) {
  Serial.write(data, size);
}


/* code */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("UHF Reader");
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  Serial.println("Single polling:");

  Serial.println("Receiving:");

  bool have_card = RFID.pollingOnce_have_card();
  if (have_card) {
    //led on
    Serial.println("card found");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);  //open the gate
    digitalWrite(LED_BUILTIN, LOW); 
  }
  delay(30);
  Serial.println();
}