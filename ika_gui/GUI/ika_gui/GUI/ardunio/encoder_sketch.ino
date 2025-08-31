// encoder_sketch.ino
unsigned long lastMillis = 0;
int encL = 0;
int encR = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Simüle edilmiş enkoder değerleri (gerçekte interrupt ile okunacak)
  if (millis() - lastMillis > 200) {
    lastMillis = millis();
    encL += 1;
    encR += 2;
    // Seri format: ENCL:123,ENCR:456
    Serial.print("ENCL:"); Serial.print(encL);
    Serial.print(",ENCR:"); Serial.println(encR);
  }

  // Gelen komutları oku (FORWARD, BACKWARD, LEFT, RIGHT, STOP)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.print("CMD_RECV:"); Serial.println(cmd);
      // Burada motora yön verebilirsin
      // örn: if (cmd == "FORWARD") { /* motorları ileri çalıştır */ }
    }
  }
}
