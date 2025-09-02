#include <Servo.h>

Servo panServo;
Servo tiltServo;
int laserPin = 13;

void setup() {
  Serial.begin(9600);
  panServo.attach(9);    // Pan servo
  tiltServo.attach(10);  // Tilt servo
  pinMode(laserPin, OUTPUT);
  panServo.write(90);    // başlangıç
  tiltServo.write(90);
  
  Serial.println("BARLAS Arduino Ready");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("MOVE")) {
      int comma1 = cmd.indexOf(',');
      int comma2 = cmd.lastIndexOf(',');
      int pan = cmd.substring(comma1 + 1, comma2).toInt();
      int tilt = cmd.substring(comma2 + 1).toInt();
      panServo.write(pan);
      tiltServo.write(tilt);
      Serial.println("Moved");
    }
    else if (cmd == "LASER,ON") {
      digitalWrite(laserPin, HIGH);
      Serial.println("Laser ON");
    }
    else if (cmd == "LASER,OFF") {
      digitalWrite(laserPin, LOW);
      Serial.println("Laser OFF");
    }
    else if (cmd == "TEST") {
      Serial.println("OK");
    }
    else if (cmd == "STATUS") {
      Serial.println("READY");
    }
  }
}
