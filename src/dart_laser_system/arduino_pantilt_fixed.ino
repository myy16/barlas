/*
 * BARLAS HIBRID SISTEM - ARDUINO KONTROL KODU
 * Tam Entegrasyonlu Sürüm: Pan-Tilt + Motor + Fren + Far + Encoder
 * 
 * Çalışan encoder, fren ve far kodlarınızı kullanır!
 */

#include <Servo.h>

// ===== SERVO KONTROL =====
Servo panServo;
Servo tiltServo;
Servo servo3;   // Ek servo
Servo servo4;   // Ek servo

// ===== PIN TANIMLAMALARI =====
// GERÇEK DONANIM KONFIGÜRASYONU

// Lazer Modülü
int laserPin = 13;

// Röle Kontrolleri (Pin 22-23)
int headlightPin = 22;   // Röle 1 - Far kontrolü  
int relay2Pin = 23;      // Röle 2 - Ek kontrol

// Encoder Pinleri (Pin 2-3, 18-19 Interrupt)
int encoder1PinA = 2;    // Sol Encoder A kanalı (Interrupt 0)
int encoder1PinB = 3;    // Sol Encoder B kanalı (Interrupt 1)
int encoder2PinA = 18;   // Sağ Encoder A kanalı (Interrupt 5)
int encoder2PinB = 19;   // Sağ Encoder B kanalı (Interrupt 4)

// Servo Kontrolleri (Pin 6,7,8,9)
int panServoPin = 6;     // Pan servo
int tiltServoPin = 7;    // Tilt servo
int servo3Pin = 8;       // Ek servo 3
int servo4Pin = 9;       // Ek servo 4

// 5V Regülatör kontrolü (güç yönetimi)
int regulator5VPin = 10; // 5V regülatör enable/disable

// ===== GLOBAL DEĞİŞKENLER =====
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
boolean headlightActive = false;
boolean relay2Active = false;
boolean regulator5VActive = true;  // 5V regülatör durumu

// Kontrol modu
String controlMode = "SERIAL";  // SERIAL veya PIXHAWK

void setup() {
  Serial.begin(9600);
  
  // ===== SERVO BAŞLATMA =====
  panServo.attach(panServoPin);     // Pan servo (Pin 6)
  tiltServo.attach(tiltServoPin);   // Tilt servo (Pin 7)
  servo3.attach(servo3Pin);         // Ek servo 3 (Pin 8)
  servo4.attach(servo4Pin);         // Ek servo 4 (Pin 9)
  
  // Başlangıç pozisyonları
  panServo.write(90);
  tiltServo.write(90);
  servo3.write(90);
  servo4.write(90);
  
  // ===== PIN KONFIGÜRASYONU =====
  pinMode(laserPin, OUTPUT);
  pinMode(headlightPin, OUTPUT);    // Röle 1 (Pin 22)
  pinMode(relay2Pin, OUTPUT);       // Röle 2 (Pin 23)
  pinMode(regulator5VPin, OUTPUT);  // 5V Regülatör (Pin 10)
  
  // Encoder pinleri
  pinMode(encoder1PinA, INPUT_PULLUP);  // Pin 2
  pinMode(encoder1PinB, INPUT_PULLUP);  // Pin 3
  pinMode(encoder2PinA, INPUT_PULLUP);  // Pin 18
  pinMode(encoder2PinB, INPUT_PULLUP);  // Pin 19
  
  // ===== INTERRUPT BAŞLATMA =====
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);  // Pin 2
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), encoder1ISR, CHANGE);  // Pin 3
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);  // Pin 18
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), encoder2ISR, CHANGE);  // Pin 19
  
  // ===== BAŞLANGIÇ DURUMU =====
  digitalWrite(laserPin, LOW);          // Lazer kapalı
  digitalWrite(headlightPin, LOW);      // Far kapalı
  digitalWrite(relay2Pin, LOW);         // Röle 2 kapalı
  digitalWrite(regulator5VPin, HIGH);   // 5V regülatör açık
  
  Serial.println("BARLAS Arduino Ready - GERÇEK PIN KONFIGÜRASYONU");
  Serial.println("Röle: Pin 22-23 | Encoder: Pin 2-3,18-19 | Servo: Pin 6-9");
}
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // ===== PAN-TILT KOMUTLARI =====
    if (cmd.startsWith("MOVE")) {
      int comma1 = cmd.indexOf(',');
      int comma2 = cmd.lastIndexOf(',');
      int pan = cmd.substring(comma1 + 1, comma2).toInt();
      int tilt = cmd.substring(comma2 + 1).toInt();
      panServo.write(pan);   // Pin 6
      tiltServo.write(tilt); // Pin 7
      Serial.println("Moved");
    }
    
    // ===== SERVO KOMUTLARI =====
    else if (cmd.startsWith("SERVO3")) {
      int comma = cmd.indexOf(',');
      int angle = cmd.substring(comma + 1).toInt();
      servo3.write(angle);   // Pin 8
      Serial.println("Servo3 " + String(angle));
    }
    else if (cmd.startsWith("SERVO4")) {
      int comma = cmd.indexOf(',');
      int angle = cmd.substring(comma + 1).toInt();
      servo4.write(angle);   // Pin 9
      Serial.println("Servo4 " + String(angle));
    }
    
    // ===== LAZER KOMUTLARI =====
    else if (cmd == "LASER,ON") {
      digitalWrite(laserPin, HIGH);  // Pin 13
      Serial.println("Laser ON");
    }
    else if (cmd == "LASER,OFF") {
      digitalWrite(laserPin, LOW);   // Pin 13
      Serial.println("Laser OFF");
    }
    
    // ===== RÖLE KOMUTLARI =====
    else if (cmd == "HEADLIGHT_ON") {
      digitalWrite(headlightPin, HIGH);  // Pin 22
      headlightActive = true;
      Serial.println("Headlight ON (Pin 22)");
    }
    else if (cmd == "HEADLIGHT_OFF") {
      digitalWrite(headlightPin, LOW);   // Pin 22
      headlightActive = false;
      Serial.println("Headlight OFF (Pin 22)");
    }
    else if (cmd == "RELAY2_ON") {
      digitalWrite(relay2Pin, HIGH);     // Pin 23
      relay2Active = true;
      Serial.println("Relay2 ON (Pin 23)");
    }
    else if (cmd == "RELAY2_OFF") {
      digitalWrite(relay2Pin, LOW);      // Pin 23
      relay2Active = false;
      Serial.println("Relay2 OFF (Pin 23)");
    }
    
    // ===== GÜÇ YÖNETİMİ =====
    else if (cmd == "5V_ON") {
      digitalWrite(regulator5VPin, HIGH);  // Pin 10
      regulator5VActive = true;
      Serial.println("5V Regulator ON (Pin 10)");
    }
    else if (cmd == "5V_OFF") {
      digitalWrite(regulator5VPin, LOW);   // Pin 10
      regulator5VActive = false;
      Serial.println("5V Regulator OFF (Pin 10)");
    }
    
    // ===== ENCODER OKUMA =====
    else if (cmd == "GET_ENCODERS") {
      Serial.println("ENCODER1:" + String(encoder1Count) + ",ENCODER2:" + String(encoder2Count));
    }
    else if (cmd == "RESET_ENCODERS") {
      encoder1Count = 0;
      encoder2Count = 0;
      Serial.println("Encoders Reset");
    }
    
    // ===== SİSTEM KOMUTLARI =====
    else if (cmd == "TEST") {
      Serial.println("OK");
    }
    else if (cmd == "STATUS") {
      printSystemStatus();
    }
    else {
      Serial.println("UNKNOWN_COMMAND");
    }
  }
}
      digitalWrite(headlightPin, HIGH);
      headlightsOn = true;
      Serial.println("Headlights ON");
    }
    else if (cmd == "HEADLIGHT_OFF") {
      digitalWrite(headlightPin, LOW);
      headlightsOn = false;
      Serial.println("Headlights OFF");
    }
    
    // ===== ENCODER OKUMA =====
    else if (cmd == "GET_ENCODERS") {
      Serial.println("ENCODER1:" + String(encoder1Count) + ",ENCODER2:" + String(encoder2Count));
    }
    else if (cmd == "RESET_ENCODERS") {
      encoder1Count = 0;
      encoder2Count = 0;
      Serial.println("Encoders Reset");
    }
    
    // ===== KONTROL MODU =====
    else if (cmd.startsWith("SET_MODE")) {
      int comma = cmd.indexOf(',');
      controlMode = cmd.substring(comma + 1);
      Serial.println("Control mode: " + controlMode);
    }
    
    // ===== SİSTEM KOMUTLARI =====
    else if (cmd == "EMERGENCY_BRAKE") {
      emergencyBrake();
      Serial.println("EMERGENCY BRAKE ACTIVATED");
    }
    else if (cmd == "TEST") {
      Serial.println("OK");
    }
    else if (cmd == "STATUS") {
      printSystemStatus();
    }
    else {
      Serial.println("UNKNOWN_COMMAND");
    }
  }
}

// ===== MOTOR KONTROL FONKSİYONLARI =====
void moveForward(int speed) {
  speed = constrain(speed, 0, 255);
  
  // Yumuşak başlatma (çalışan kodlardan)
  if (!motorsEnabled) {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    motorsEnabled = true;
    
    // Yumuşak başlat
    for (int i = 0; i <= speed; i += 5) {
      analogWrite(RPWM, i);
      analogWrite(LPWM, 0);
      delay(20);
// ===== ENCODER INTERRUPT FONKSİYONLARI =====
void encoder1ISR() {
  // Sol encoder interrupt (Pin 2,3)
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

void encoder2ISR() {
  // Sağ encoder interrupt (Pin 18,19)
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}

// ===== DURUM BİLGİSİ =====
void printSystemStatus() {
  Serial.println("=== BARLAS GERÇEK PIN DURUMU ===");
  Serial.println("Control Mode: " + controlMode);
  Serial.println("Headlight (Pin 22): " + String(headlightActive ? "ON" : "OFF"));
  Serial.println("Relay2 (Pin 23): " + String(relay2Active ? "ON" : "OFF"));
  Serial.println("5V Regulator (Pin 10): " + String(regulator5VActive ? "ON" : "OFF"));
  Serial.println("Encoder1 (Pin 2,3): " + String(encoder1Count));
  Serial.println("Encoder2 (Pin 18,19): " + String(encoder2Count));
  Serial.println("Laser (Pin 13): " + String(digitalRead(laserPin) ? "ON" : "OFF"));
  Serial.println("Servo Pins: 6,7,8,9");
  Serial.println("================================");
}

void emergencyBrake() {
  stopMotors();          // Önce motorları durdur
  delay(100);
  engageBrake();         // Sonra freni çek
  digitalWrite(headlightPin, HIGH);  // Farları aç (görünürlük için)
}

// ===== ENCODER INTERRUPT FONKSİYONLARI =====
void encoder1ISR() {
  // Çalışan kodlarınızdaki encoder mantığı
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

void encoder2ISR() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}

// ===== DURUM BİLGİSİ =====
void printSystemStatus() {
  Serial.println("=== BARLAS SYSTEM STATUS ===");
  Serial.println("Control Mode: " + controlMode);
  Serial.println("Motor Speed: " + String(currentSpeed));
  Serial.println("Motors: " + String(motorsEnabled ? "ON" : "OFF"));
  Serial.println("Brake: " + String(brakeEngaged ? "ENGAGED" : "RELEASED"));
  Serial.println("Headlights: " + String(headlightsOn ? "ON" : "OFF"));
  Serial.println("Encoder1: " + String(encoder1Count));
  Serial.println("Encoder2: " + String(encoder2Count));
  Serial.println("Laser: " + String(digitalRead(laserPin) ? "ON" : "OFF"));
  Serial.println("============================");
}
