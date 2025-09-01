#include <Servo.h>

Servo panServo;
Servo tiltServo;
int laserPin = 13;

void setup() {
  pinMode(laserPin, OUTPUT);  // Pin’i OUTPUT yap
  digitalWrite(laserPin, HIGH); // Başlangıçta lazer kapalı
  delay(10); // Pin stabilitesini artırmak için küçük gecikme

  Serial.begin(9600);
  panServo.attach(9);   // Pan servo
  tiltServo.attach(10); // Tilt servo
  panServo.write(0);   // başlangıç
  tiltServo.write(0);
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
    else if (cmd == "LASER,OFF") {
      digitalWrite(laserPin, HIGH);
      Serial.println("Laser OFF");
    }
    else if (cmd == "LASER,ON") {
      digitalWrite(laserPin, LOW);
      Serial.println("Laser ON");
    }

  }
}
