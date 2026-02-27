#include <Servo.h>
Servo myServo;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(8, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int reading1 = analogRead(A0);
  delayMicroseconds(400);
  int reading2 = analogRead(A0);
  delayMicroseconds(400);
  int reading3 = analogRead(A0);
  int filtered = max(reading1, max(reading2, reading3));
  Serial.println(filtered-512);
  delay(100);

}
