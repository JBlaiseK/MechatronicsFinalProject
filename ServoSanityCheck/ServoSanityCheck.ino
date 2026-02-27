#include <Servo.h>
Servo myServo;
myServo.attach(5);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
if (counter > 10) {
    Serial.println(high-low);
    counter = 0;
    for (int pos = 0; pos <= 180; pos += 1) {
      myServo.write(pos);              // Tell servo to go to position
      delay(15);                       // Wait 15ms for the servo to reach the position
    }
    for (int pos = 180; pos >= 0; pos -= 1) {
      myServo.write(pos);              // Tell servo to go to position
      delay(15);                       // Wait 15ms for the servo to reach the position
    }
  }
}
