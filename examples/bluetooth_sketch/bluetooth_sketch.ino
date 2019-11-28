#include <SoftwareSerial.h>

// SoftwareSerial for servo
SoftwareSerial serial_servo(5, 6);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  serial_servo.begin(9600);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Serial Monitor");
  delay(1000);

  if (serial_servo.available() > 0) {
    Serial.println("Servo Serial available");

    char c = serial_servo.read();
    Serial.println(c);

    if(c == 'w') {
      Serial.println("Serial Monitor - Forward");
    } else if (c == 'x') {
      Serial.println("Serial Monitor - Back");
    } else if (c == 's') {
      Serial.println("Serial Monitor - Stop");
    } else if (c == 'a') {
      Serial.println("Serial Monitor - Left");
    } else if (c == 'd') {
      Serial.println("Serial Monitor - Right");
    } else if (c == 'q') {
      Serial.println("Serial Monitor - Stop turn");
    } else {
      Serial.println("Serial Monitor - " + c);
    }

    // wait for a second
    delay(1000);
  }
}
