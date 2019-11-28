#include <SoftwareSerial.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Serial Monitor");
  delay(1000);

  if (Serial.available() > 0) {
    Serial.println("Serial available");

    char c = Serial.read();
    Serial.println(c);

    if(c == 97) {
      Serial.println("Serial Monitor - HIGH");
      // turn the LED off by making the voltage LOW
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      Serial.println("Serial Monitor - LOW");
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    }

    // wait for a second
    delay(1000);
  }
}
