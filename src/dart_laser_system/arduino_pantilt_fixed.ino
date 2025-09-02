/*
 * BARLAS HIBRID SISTEM - ARDUINO KONTROL KODU
 * Tam Entegrasyonlu Sürüm: Pan-Tilt + Fren Servo + Röle + Encoder
 * 
 * Arkadaşların çalışan fren servo + röle kodları entegre edildi!
 */

#include <Servo.h>

// ===== SERVO KONTROL =====
Servo panServo;
Servo tiltServo;

// --- Fren Servo tanımları (Arkadaşların kodu) ---
Servo frenServo1;
Servo frenServo2;

// ===== PIN TANIMLAMALARI =====
// GERÇEK DONANIM KONFIGÜRASYONU + Arkadaşların pin konfigürasyonu

// Lazer Modülü
int laserPin = 13;

// Pan-Tilt Servo Kontrolleri 
int panServoPin = 6;     // Pan servo
int tiltServoPin = 7;    // Tilt servo

// --- Fren Servo Pinleri (Arkadaşların kodu) ---
const int servoPin1 = 9;   // Fren servo 1
const int servoPin2 = 10;  // Fren servo 2
const int pwmServoPin = 3; // PWM giriş (Servo kontrol)

// --- Röle Pinleri (Arkadaşların kodu) ---
const int roleOut = 8;     // Röle çıkış
const int pwmRolePin = 4;  // PWM giriş (Röle kontrol)

// Encoder Pinleri (Pin 2-3, 18-19 Interrupt) - Pin 2,3 Fren ile çakışıyor, 18-19 kullanılacak
int encoder1PinA = 18;   // Sol Encoder A kanalı (Interrupt 5)
int encoder1PinB = 19;   // Sol Encoder B kanalı (Interrupt 4)
int encoder2PinA = 20;   // Sağ Encoder A kanalı (Interrupt 3) - Yeni pin
int encoder2PinB = 21;   // Sağ Encoder B kanalı (Interrupt 2) - Yeni pin

// 5V Regülatör kontrolü (güç yönetimi)
int regulator5VPin = 22; // 5V regülatör enable/disable (pin değişti)

// --- Fren Servo Değişkenleri (Arkadaşların kodu) ---
int frenCekAci1 = -40;
int frenCekAci2 = -40;
int servoOffset1, servoOffset2;

// --- PWM ölçüm değişkenleri (Arkadaşların kodu) ---
volatile unsigned long servoRiseTime = 0;
volatile unsigned long servoPulse = 0;
volatile unsigned long roleRiseTime = 0;
volatile unsigned long rolePulse = 0;

// ===== GLOBAL DEĞİŞKENLER =====
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
boolean regulator5VActive = true;  // 5V regülatör durumu

// Kontrol modu
String controlMode = "SERIAL";  // SERIAL veya PIXHAWK

void setup() {
  Serial.begin(115200);
  Serial.println("BARLAS HIBRID SISTEM BAŞLADI!");
  
  // ===== SERVO BAĞLANTILARI =====
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
  
  // --- Fren servo bağlantıları (Arkadaşların kodu) ---
  frenServo1.attach(servoPin1);
  frenServo2.attach(servoPin2);
  
  // Servo başlangıç kalibrasyonu
  servoOffset1 = 90 + frenCekAci1;
  servoOffset2 = 90 + frenCekAci2;
  
  frenServo1.write(90); // Nötr pozisyon
  frenServo2.write(90);
  
  // ===== PIN MODU AYARLARI =====
  pinMode(laserPin, OUTPUT);
  pinMode(regulator5VPin, OUTPUT);
  pinMode(roleOut, OUTPUT);
  
  // PWM giriş pinleri
  pinMode(pwmServoPin, INPUT);
  pinMode(pwmRolePin, INPUT);
  
  // ===== ENCODER INTERRUPT AYARLARI =====
  // Encoder 1 (Sol) - Pin 18,19
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);
  
  // Encoder 2 (Sağ) - Pin 20,21
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), updateEncoder2, CHANGE);
  
  // --- PWM Interrupt ayarları (Arkadaşların kodu) ---
  attachInterrupt(digitalPinToInterrupt(pwmServoPin), servoRiseInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(pwmServoPin), servoFallInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(pwmRolePin), roleRiseInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(pwmRolePin), roleFallInterrupt, FALLING);
  
  // ===== BAŞLANGIÇ AYARLARI =====
  digitalWrite(laserPin, LOW);
  digitalWrite(regulator5VPin, HIGH); // 5V regülatör aktif
  digitalWrite(roleOut, LOW);         // Röle kapalı
  
  // Pan-Tilt başlangıç pozisyonu
  panServo.write(90);
  tiltServo.write(90);
  
  delay(1000);
  Serial.println("SİSTEM HAZIR!");
  Serial.println("KOMUTLAR: laser_on, laser_off, move_pan_90, fren_uygula, fren_birak, role_on, role_off");
}
// ===== PWM INTERRUPT FONKSİYONLARI (Arkadaşların Kodu) =====

// Servo PWM interrupt fonksiyonları
void servoRiseInterrupt() {
  servoRiseTime = micros();
}

void servoFallInterrupt() {
  servoPulse = micros() - servoRiseTime;
}

// Röle PWM interrupt fonksiyonları  
void roleRiseInterrupt() {
  roleRiseTime = micros();
}

void roleFallInterrupt() {
  rolePulse = micros() - roleRiseTime;
}

// ===== ENCODER INTERRUPT FONKSİYONLARI =====

// Encoder 1 (Sol) interrupt - Pin 18
void updateEncoder1() {
  int MSB = digitalRead(encoder1PinA);
  int LSB = digitalRead(encoder1PinB);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (encoder1Count << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder1Count++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder1Count--;
}

// Encoder 2 (Sağ) interrupt - Pin 20  
void updateEncoder2() {
  int MSB = digitalRead(encoder2PinA);
  int LSB = digitalRead(encoder2PinB);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (encoder2Count << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder2Count++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder2Count--;
}

void loop() {
  // === PWM OKUMA VE FİLTRELEME (Arkadaşların kodu) ===
  static unsigned long lastServoTime = 0;
  static unsigned long lastRoleTime = 0;
  
  // Servo PWM oku (her 20ms)
  if (millis() - lastServoTime > 20) {
    unsigned long pulseWidth = servoPulse; // Atom değer okuma
    
    if (pulseWidth > 800 && pulseWidth < 2200) { // Geçerli PWM aralığı
      int servoAngle = map(pulseWidth, 1000, 2000, 0, 180);
      
      // Fren servo kontrolü
      if (servoAngle < 85) { // Fren uygula
        frenServo1.write(servoOffset1);
        frenServo2.write(servoOffset2);
      } else if (servoAngle > 95) { // Fren bırak
        frenServo1.write(90);
        frenServo2.write(90);
      }
      // 85-95 arası nötr bölge - değişiklik yapma
    }
    lastServoTime = millis();
  }
  
  // Röle PWM oku (her 50ms)
  if (millis() - lastRoleTime > 50) {
    unsigned long pulseWidth = rolePulse; // Atom değer okuma
    
    if (pulseWidth > 800 && pulseWidth < 2200) { // Geçerli PWM aralığı
      int roleValue = map(pulseWidth, 1000, 2000, 0, 180);
      
      if (roleValue > 90) {
        digitalWrite(roleOut, HIGH); // Röle aç
      } else {
        digitalWrite(roleOut, LOW);  // Röle kapat
      }
    }
    lastRoleTime = millis();
  }
  
  // === SERİ KOMUT İŞLEME ===
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
      Serial.println("Pan-Tilt moved: " + String(pan) + "," + String(tilt));
    }
    
    // ===== LAZER KOMUTLARI =====
    else if (cmd == "LASER,ON" || cmd == "laser_on") {
      digitalWrite(laserPin, HIGH);
      Serial.println("Laser ON");
    }
    else if (cmd == "LASER,OFF" || cmd == "laser_off") {
      digitalWrite(laserPin, LOW);
      Serial.println("Laser OFF");
    }
    
    // ===== MANUEL FREN KOMUTLARI =====
    else if (cmd == "fren_uygula") {
      frenServo1.write(servoOffset1);
      frenServo2.write(servoOffset2);
      Serial.println("Manuel fren uygulandı");
    }
    else if (cmd == "fren_birak") {
      frenServo1.write(90);
      frenServo2.write(90);
      Serial.println("Manuel fren bırakıldı");
    }
    
    // ===== MANUEL RÖLE KOMUTLARI =====
    else if (cmd == "role_on") {
      digitalWrite(roleOut, HIGH);
      Serial.println("Röle açıldı");
    }
    else if (cmd == "role_off") {
      digitalWrite(roleOut, LOW);
      Serial.println("Röle kapatıldı");
    }
    
    // ===== ENCODER OKUMA =====
    else if (cmd == "encoder_read") {
      Serial.print("Encoder1: ");
      Serial.print(encoder1Count);
      Serial.print(" | Encoder2: ");
      Serial.println(encoder2Count);
    }
    else if (cmd == "encoder_reset") {
      encoder1Count = 0;
      encoder2Count = 0;
      Serial.println("Encoders reset");
    }
    
    // ===== PWM DURUM OKUMA =====
    else if (cmd == "pwm_status") {
      Serial.print("Servo PWM: ");
      Serial.print(servoPulse);
      Serial.print("μs | Role PWM: ");
      Serial.print(rolePulse);
      Serial.println("μs");
    }
    
    // ===== SISTEM BİLGİLERİ =====
    else if (cmd == "status") {
      Serial.println("=== BARLAS SİSTEM DURUM ===");
      Serial.print("Encoder1: "); Serial.println(encoder1Count);
      Serial.print("Encoder2: "); Serial.println(encoder2Count);
      Serial.print("Servo PWM: "); Serial.print(servoPulse); Serial.println("μs");
      Serial.print("Role PWM: "); Serial.print(rolePulse); Serial.println("μs");
      Serial.println("========================");
    }
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
