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
Servo brakeServo;  // Fren servosu

// ===== PIN TANIMLAMALARI =====
// Lazer Modülü
int laserPin = 13;

// BTS7960 Motor Sürücü Pinleri
int R_EN = 2;      // Sağ Enable (PWM için)
int L_EN = 3;      // Sol Enable (PWM için)
int RPWM = 4;      // Sağ PWM
int LPWM = 5;      // Sol PWM
int R_IS = A0;     // Sağ akım sensörü
int L_IS = A1;     // Sol akım sensörü

// Far Kontrol (Röle)
int headlightPin = 7;

// Fren Servo
int brakePin = 6;

// Encoder Pinleri (Interrupt)
int encoder1PinA = 21;  // Encoder 1 A kanalı (Interrupt 0)
int encoder1PinB = 20;  // Encoder 1 B kanalı (Interrupt 1)
int encoder2PinA = 19;  // Encoder 2 A kanalı (Interrupt 4)  
int encoder2PinB = 18;  // Encoder 2 B kanalı (Interrupt 5)

// ===== GLOBAL DEĞİŞKENLER =====
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
int currentSpeed = 0;
boolean motorsEnabled = false;
boolean headlightsOn = false;
boolean brakeEngaged = false;

// Kontrol modu
String controlMode = "SERIAL";  // SERIAL veya PIXHAWK

void setup() {
  Serial.begin(9600);
  
  // ===== SERVO BAŞLATMA =====
  panServo.attach(9);     // Pan servo
  tiltServo.attach(10);   // Tilt servo
  brakeServo.attach(brakePin);  // Fren servo
  
  // Başlangıç pozisyonları
  panServo.write(90);
  tiltServo.write(90);
  brakeServo.write(0);    // Fren açık (0 = fren açık)
  
  // ===== PIN KONFIGÜRASYONU =====
  pinMode(laserPin, OUTPUT);
  pinMode(headlightPin, OUTPUT);
  
  // BTS7960 Motor Sürücü
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_IS, INPUT);
  pinMode(L_IS, INPUT);
  
  // Encoder pinleri
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  
  // ===== INTERRUPT BAŞLATMA =====
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);
  
  // ===== BAŞLANGIÇ DURUMU =====
  digitalWrite(laserPin, LOW);
  digitalWrite(headlightPin, LOW);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  
  Serial.println("BARLAS Arduino Ready - Full System");
  Serial.println("Controls: Pan-Tilt, Motor, Brake, Headlight, Encoder");
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
      panServo.write(pan);
      tiltServo.write(tilt);
      Serial.println("Moved");
    }
    
    // ===== LAZER KOMUTLARI =====
    else if (cmd == "LASER,ON") {
      digitalWrite(laserPin, HIGH);
      Serial.println("Laser ON");
    }
    else if (cmd == "LASER,OFF") {
      digitalWrite(laserPin, LOW);
      Serial.println("Laser OFF");
    }
    
    // ===== MOTOR KOMUTLARI =====
    else if (cmd.startsWith("MOTOR_FORWARD")) {
      int comma = cmd.indexOf(',');
      int speed = cmd.substring(comma + 1).toInt();
      moveForward(speed);
      Serial.println("Motor Forward " + String(speed));
    }
    else if (cmd.startsWith("MOTOR_BACKWARD")) {
      int comma = cmd.indexOf(',');
      int speed = cmd.substring(comma + 1).toInt();
      moveBackward(speed);
      Serial.println("Motor Backward " + String(speed));
    }
    else if (cmd == "MOTOR_STOP") {
      stopMotors();
      Serial.println("Motors Stopped");
    }
    
    // ===== FREN KOMUTLARI =====
    else if (cmd == "BRAKE_ON") {
      engageBrake();
      Serial.println("Brake ON");
    }
    else if (cmd == "BRAKE_OFF") {
      releaseBrake();
      Serial.println("Brake OFF");
    }
    
    // ===== FAR KOMUTLARI =====
    else if (cmd == "HEADLIGHT_ON") {
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
    }
  } else {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  }
  
  currentSpeed = speed;
}

void moveBackward(int speed) {
  speed = constrain(speed, 0, 255);
  
  if (!motorsEnabled) {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    motorsEnabled = true;
  }
  
  analogWrite(RPWM, 0);
  analogWrite(LPWM, speed);
  currentSpeed = -speed;
}

void stopMotors() {
  // Yumuşak durdurma
  int currentPWM = abs(currentSpeed);
  for (int i = currentPWM; i >= 0; i -= 10) {
    if (currentSpeed > 0) {
      analogWrite(RPWM, i);
    } else {
      analogWrite(LPWM, i);
    }
    delay(10);
  }
  
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  motorsEnabled = false;
  currentSpeed = 0;
}

// ===== FREN KONTROL FONKSİYONLARI =====
void engageBrake() {
  brakeServo.write(90);  // Fren aç (90 derece)
  brakeEngaged = true;
}

void releaseBrake() {
  brakeServo.write(0);   // Fren kapat (0 derece)
  brakeEngaged = false;
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
