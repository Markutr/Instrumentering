#include <Servo.h>
#include <PID_v1.h>
Servo myServo;
Servo myServo2;

void setup() {
  // put your setup code here, to run once:
  myServo.attach(3);
  myServo2.attach(4);
}

void loop() {
  // put your main code here, to run repeatedly:
  myServo.write(10);
  delay(1000);
  myServo2.write(10);
  delay(1000);
  myServo.write(90);
  delay(1000);
  myServo2.write(90);
  delay(1000);
  myServo.write(170);
  delay(1000);
  myServo2.write(170);
  delay(1000);
}
