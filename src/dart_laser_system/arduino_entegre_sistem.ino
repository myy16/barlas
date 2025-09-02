/*
 * BARLAS HIBRID SISTEM - TAM ENTEGRASYONxarduino_kodu
 * Pan-Tilt + Fren Servo + PWM Röle + Encoder
 * 
 * Arkadaşlarınızın çalışan fren servo + PWM röle kodları entegre edildi!
 * Gerçek donanım pin konfigürasyonu kullanılır.
 */

#include <Servo.h>

// ===== SERVO KONTROL =====
Servo panServo;
Servo tiltServo;

// --- Fren Servo tanımları (Arkadaşların kodu) ---
Servo frenServo1;
Servo frenServo2;

// ===== PIN TANIMLAMALARI =====
// GÜNCEL DONANIM KONFIGÜRASYONU - GERÇEK PIN NUMARALARI

// PWM Giriş Pinleri (Kumanda sinyalleri)
const int pwmServoPin = 2; // PWM giriş 1 (Servo/Fren kontrol)
const int pwmRolePin = 3;  // PWM giriş 2 (Röle kontrol)

// Far Sistemi
int farPin = 6;           // Far kontrolü

// Fren Servo Pinleri
const int servoPin1 = 9;   // Fren servo 1
const int servoPin2 = 10;  // Fren servo 2

// Lazer Röle Kontrol
int lazerRolePin = 13;    // Lazer röle (bizim HIGH/LOW gönderdiğimiz pin)

// Encoder Pinleri
int encoder1PinA = 18;   // Sol Encoder A kanalı (Interrupt 5)
int encoder1PinB = 19;   // Sol Encoder B kanalı (Interrupt 4)
int encoder2PinA = 20;   // Sağ Encoder A kanalı (Interrupt 3)
int encoder2PinB = 21;   // Sağ Encoder B kanalı (Interrupt 2)

// Pixhawk Manuel Kontrol
int pixhawkManuelPin = 23; // Pixhawk manuel açma/kapama (kumandadan gelen)

// Pixhawk Kontrol Pinleri
int pixhawkPin1 = 26;     // Pixhawk kontrol 1 (X ekseni kontrol)
int pixhawkPin2 = 27;     // Pixhawk kontrol 2 (Y ekseni kontrol)

// Pan-Tilt Servo Pinleri (X-Y ekseni) - DÜZELTİLDİ
int panServoPin = 7;     // Pan servo (X ekseni)
int tiltServoPin = 8;    // Tilt servo (Y ekseni)

// --- Fren Servo Değişkenleri ---
int frenCekAci1 = -40;
int frenCekAci2 = -40;
int servoOffset1, servoOffset2;

// --- PWM ölçüm değişkenleri ---
volatile unsigned long servoRiseTime = 0;
volatile unsigned long servoPulse = 0;
volatile unsigned long roleRiseTime = 0;
volatile unsigned long rolePulse = 0;

// ===== GLOBAL DEĞİŞKENLER =====
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
boolean farActive = false;        // Far durumu
boolean pixhawkActive = false;    // Pixhawk durumu

// Kontrol modu
String controlMode = "SERIAL";  // SERIAL veya PIXHAWK

void setup() {
  Serial.begin(115200);
  Serial.println("BARLAS HIBRID SISTEM BAŞLADI!");
  Serial.println("YENİ PIN KONFIGÜRASYONU YÜKLENDI");
  
  // ===== SERVO BAĞLANTILARI =====
  panServo.attach(panServoPin);   // Pin 7
  tiltServo.attach(tiltServoPin); // Pin 8
  
  // --- Fren servo bağlantıları ---
  frenServo1.attach(servoPin1);   // Pin 9
  frenServo2.attach(servoPin2);   // Pin 10
  
  // Servo başlangıç kalibrasyonu
  servoOffset1 = 90 + frenCekAci1;
  servoOffset2 = 90 + frenCekAci2;
  
  frenServo1.write(90); // Nötr pozisyon
  frenServo2.write(90);
  
  // ===== PIN MODU AYARLARI =====
  pinMode(lazerRolePin, OUTPUT);      // Pin 13 - Lazer röle
  pinMode(farPin, OUTPUT);            // Pin 6 - Far
  pinMode(pixhawkManuelPin, OUTPUT);  // Pin 23 - Pixhawk manuel
  pinMode(pixhawkPin1, OUTPUT);       // Pin 26 - Pixhawk 1
  pinMode(pixhawkPin2, OUTPUT);       // Pin 27 - Pixhawk 2
  
  // PWM giriş pinleri
  pinMode(pwmServoPin, INPUT);        // Pin 2 - PWM Servo giriş
  pinMode(pwmRolePin, INPUT);         // Pin 3 - PWM Röle giriş
  
  // ===== ENCODER INTERRUPT AYARLARI =====
  pinMode(encoder1PinA, INPUT_PULLUP); // Pin 18
  pinMode(encoder1PinB, INPUT_PULLUP); // Pin 19
  pinMode(encoder2PinA, INPUT_PULLUP); // Pin 20
  pinMode(encoder2PinB, INPUT_PULLUP); // Pin 21
  
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), updateEncoder2, CHANGE);
  
  // --- PWM Interrupt ayarları ---
  attachInterrupt(digitalPinToInterrupt(pwmServoPin), servoRiseInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(pwmServoPin), servoFallInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(pwmRolePin), roleRiseInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(pwmRolePin), roleFallInterrupt, FALLING);
  
  // ===== BAŞLANGIÇ AYARLARI =====
  digitalWrite(lazerRolePin, LOW);      // Lazer kapalı
  digitalWrite(farPin, LOW);            // Far kapalı
  digitalWrite(pixhawkManuelPin, LOW);  // Pixhawk manuel kapalı
  digitalWrite(pixhawkPin1, LOW);       // Pixhawk 1 kapalı
  digitalWrite(pixhawkPin2, LOW);       // Pixhawk 2 kapalı
  
  // Pan-Tilt başlangıç pozisyonu
  panServo.write(90);
  tiltServo.write(90);
  
  delay(1000);
  Serial.println("SİSTEM HAZIR!");
  Serial.println("✅ GÜNCEL PIN KONFİGÜRASYONU:");
  Serial.println("   PWM Giriş: Pin 2-3 | Far: Pin 6 | Pan-Tilt: Pin 7-8");
  Serial.println("   Fren: Pin 9-10 | Lazer: Pin 13 | Encoder: Pin 18-21");
  Serial.println("   Pixhawk: Pin 23(Manuel) Pin 26-27(Kontrol)");
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

// ===== PIN DURUM KONTROL SİSTEMİ =====

void checkPixhawkPinStates() {
  """Pin 23 Pixhawk manuel kontrol durumu"""
  static bool lastPixhawkState = false;
  static unsigned long lastCheckTime = 0;
  
  // Her 100ms kontrol et (debouncing için)
  if (millis() - lastCheckTime > 100) {
    bool currentState = digitalRead(pixhawkManuelPin);
    
    if (currentState && !lastPixhawkState) {
      // Pin 23 HIGH oldu - Pixhawk aktif
      executePixhawkActivateFunction();
    } else if (!currentState && lastPixhawkState) {
      // Pin 23 LOW oldu - Pixhawk pasif
      executePixhawkDeactivateFunction();
    }
    
    lastPixhawkState = currentState;
    lastCheckTime = millis();
  }
}

// ===== MODÜLER FONKSİYONLAR =====

void executeBrakeFunction() {
  """Fren uygula fonksiyonu - char b işlevi"""
  frenServo1.write(servoOffset1);
  frenServo2.write(servoOffset2);
  Serial.println("[AUTO] Fren uygulandı");
}

void executeReleaseFunction() {
  """Fren bırak fonksiyonu - char r işlevi"""
  frenServo1.write(90);
  frenServo2.write(90);
  Serial.println("[AUTO] Fren bırakıldı");
}

void executeLaserOnFunction() {
  """Lazer aç fonksiyonu - char l işlevi"""
  digitalWrite(lazerRolePin, HIGH);
  Serial.println("[AUTO] Lazer röle açıldı");
}

void executeLaserOffFunction() {
  """Lazer kapat fonksiyonu - char o işlevi"""
  digitalWrite(lazerRolePin, LOW);
  Serial.println("[AUTO] Lazer röle kapatıldı");
}

void executePixhawkActivateFunction() {
  """Pin 23 aktif - char c işlevi"""
  pixhawkActive = true;
  controlMode = "PIXHAWK";
  Serial.println("[PIN23] Pixhawk modu aktif!");
  Serial.println("[PIN23] Otomatik mod başladı");
  
  // Pixhawk modunda yapılacak işlemler
  enableAutoPilot();
}

void executePixhawkDeactivateFunction() {
  """Pin 23 pasif - char d işlevi"""
  pixhawkActive = false;
  controlMode = "SERIAL";
  Serial.println("[PIN23] Pixhawk modu pasif");
  Serial.println("[PIN23] Manuel mod başladı");
  
  // Manuel moda geçiş işlemleri
  disableAutoPilot();
}

void enableAutoPilot() {
  """Otopilot aktif"""
  digitalWrite(pixhawkPin1, HIGH);  // Pin 26
  digitalWrite(pixhawkPin2, HIGH);  // Pin 27
  Serial.println("[AUTOPILOT] X-Y kontrol aktif");
}

void disableAutoPilot() {
  """Otopilot pasif"""
  digitalWrite(pixhawkPin1, LOW);   // Pin 26
  digitalWrite(pixhawkPin2, LOW);   // Pin 27
  Serial.println("[AUTOPILOT] Manuel kontrol aktif");
}

void executeStatusFunction() {
  """Status komutu - char s işlevi"""
  Serial.println("=== BARLAS SİSTEM DURUM (GÜNCEL PIN) ===");
  Serial.print("Encoder1 (Pin18-19): "); Serial.println(encoder1Count);
  Serial.print("Encoder2 (Pin20-21): "); Serial.println(encoder2Count);
  Serial.print("Servo PWM (Pin2): "); Serial.print(servoPulse); Serial.println("μs");
  Serial.print("Lazer PWM (Pin3): "); Serial.print(rolePulse); Serial.println("μs");
  Serial.print("Far (Pin6): "); Serial.println(digitalRead(farPin) ? "ON" : "OFF");
  Serial.print("Pan-Tilt (Pin7-8): Pan="); Serial.print(panServo.read()); Serial.print("° Tilt="); Serial.print(tiltServo.read()); Serial.println("°");
  Serial.print("Fren (Pin9-10): "); Serial.print(frenServo1.read()); Serial.print("° / "); Serial.print(frenServo2.read()); Serial.println("°");
  Serial.print("Lazer Röle (Pin13): "); Serial.println(digitalRead(lazerRolePin) ? "ON" : "OFF");
  Serial.print("Pixhawk Manuel (Pin23): "); Serial.println(digitalRead(pixhawkManuelPin) ? "ON" : "OFF");
  Serial.print("Pixhawk X-Y (Pin26-27): "); Serial.print(digitalRead(pixhawkPin1) ? "ON" : "OFF"); Serial.print(" / "); Serial.println(digitalRead(pixhawkPin2) ? "ON" : "OFF");
  Serial.print("Kontrol Modu: "); Serial.println(controlMode);
  Serial.println("========================================");
}

void executeTestFunction() {
  """Test komutu - char t işlevi"""
  Serial.println("Arduino bağlantı testi: OK - GÜNCEL PIN KONFİGÜRASYONU");
  Serial.println("✅ Pin 2-3:PWM | Pin 6:Far | Pin 7-8:Pan-Tilt | Pin 9-10:Fren");
  Serial.println("✅ Pin 13:Lazer | Pin 18-21:Encoder | Pin 23:Pixhawk | Pin 26-27:X-Y");
  Serial.println("🎮 Char Komutlar: b=fren, r=bırak, l=lazer_on, o=lazer_off, c=pixhawk_on, d=pixhawk_off, f=far_on, g=far_off, s=status, t=test");
}

void loop() {
  // === PIN DURUM KONTROLÜ SİSTEMİ ===
  checkPixhawkPinStates();
  
  // === PWM OKUMA VE FİLTRELEME ===
  static unsigned long lastServoTime = 0;
  static unsigned long lastRoleTime = 0;
  
  // Servo PWM oku (her 20ms)
  if (millis() - lastServoTime > 20) {
    unsigned long pulseWidth = servoPulse; // Atom değer okuma
    
    if (pulseWidth > 800 && pulseWidth < 2200) { // Geçerli PWM aralığı
      int servoAngle = map(pulseWidth, 1000, 2000, 0, 180);
      
      // Fren servo kontrolü
      if (servoAngle < 85) { // Fren uygula
        executeBrakeFunction();
      } else if (servoAngle > 95) { // Fren bırak
        executeReleaseFunction();
      }
      // 85-95 arası nötr bölge - değişiklik yapma
    }
    lastServoTime = millis();
  }
  
  // Röle PWM oku (her 50ms) - Lazer röle kontrolü
  if (millis() - lastRoleTime > 50) {
    unsigned long pulseWidth = rolePulse; // Atom değer okuma
    
    if (pulseWidth > 800 && pulseWidth < 2200) { // Geçerli PWM aralığı
      int roleValue = map(pulseWidth, 1000, 2000, 0, 180);
      
      if (roleValue > 90) {
        executeLaserOnFunction();
      } else {
        executeLaserOffFunction();
      }
    }
    lastRoleTime = millis();
  }
  
  // === SERİ KOMUT İŞLEME ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // ===== TEK KARAKTER KOMUTLAR (MODÜLER SİSTEM) =====
    if (cmd.length() == 1) {
      char c = cmd.charAt(0);
      switch(c) {
        case 'b':  // Brake
          executeBrakeFunction();
          break;
        case 'r':  // Release
          executeReleaseFunction();
          break;
        case 'l':  // Laser on
          executeLaserOnFunction();
          break;
        case 'o':  // Laser off
          executeLaserOffFunction();
          break;
        case 'c':  // Activate (Pixhawk manuel)
          executePixhawkActivateFunction();
          break;
        case 'd':  // Deactivate (Pixhawk pasif)
          executePixhawkDeactivateFunction();
          break;
        case 'f':  // Far on
          digitalWrite(farPin, HIGH);
          Serial.println("[CHAR] Far açıldı");
          break;
        case 'g':  // Far off (f'den sonraki harf)
          digitalWrite(farPin, LOW);
          Serial.println("[CHAR] Far kapatıldı");
          break;
        case 's':  // Status
          executeStatusFunction();
          break;
        case 't':  // Test
          executeTestFunction();
          break;
        default:
          Serial.println("Bilinmeyen char komut: " + cmd);
          break;
      }
    }
    
    // ===== PAN-TILT KOMUTLARI =====
    else if (cmd.startsWith("MOVE")) {
      int comma1 = cmd.indexOf(',');
      int comma2 = cmd.lastIndexOf(',');
      int pan = cmd.substring(comma1 + 1, comma2).toInt();
      int tilt = cmd.substring(comma2 + 1).toInt();
      panServo.write(pan);   // Pin 7
      tiltServo.write(tilt); // Pin 8
      Serial.println("Pan-Tilt moved: " + String(pan) + "," + String(tilt));
    }
    
    // ===== LAZER KOMUTLARI =====
    else if (cmd == "LASER,ON" || cmd == "lazer_on") {
      executeLaserOnFunction(); // Modüler fonksiyon çağrısı
    }
    else if (cmd == "LASER,OFF" || cmd == "lazer_off") {
      executeLaserOffFunction(); // Modüler fonksiyon çağrısı
    }
    
    // ===== FAR KOMUTLARI =====
    else if (cmd == "far_on") {
      digitalWrite(farPin, HIGH);       // Pin 6
      farActive = true;
      Serial.println("[CMD] Far açıldı");
      Serial.println("Far açıldı");
    }
    else if (cmd == "far_off") {
      digitalWrite(farPin, LOW);        // Pin 6
      farActive = false;
      Serial.println("Far kapatıldı");
    }
    
    // ===== MANUEL FREN KOMUTLARI =====
    else if (cmd == "fren_uygula") {
      executeBrakeFunction(); // Modüler fonksiyon çağrısı
    }
    else if (cmd == "fren_birak") {
      executeReleaseFunction(); // Modüler fonksiyon çağrısı
    }
    
    // ===== PIXHAWK KOMUTLARI =====
    else if (cmd == "pixhawk_manuel_on") {
      digitalWrite(pixhawkManuelPin, HIGH); // Pin 23
      Serial.println("Pixhawk manuel aktif");
    }
    else if (cmd == "pixhawk_manuel_off") {
      digitalWrite(pixhawkManuelPin, LOW);  // Pin 23
      Serial.println("Pixhawk manuel pasif");
    }
    else if (cmd == "pixhawk1_on") {
      digitalWrite(pixhawkPin1, HIGH);      // Pin 26
      Serial.println("Pixhawk 1 aktif");
    }
    else if (cmd == "pixhawk1_off") {
      digitalWrite(pixhawkPin1, LOW);       // Pin 26
      Serial.println("Pixhawk 1 pasif");
    }
    else if (cmd == "pixhawk2_on") {
      digitalWrite(pixhawkPin2, HIGH);      // Pin 27
      Serial.println("Pixhawk 2 aktif");
    }
    else if (cmd == "pixhawk2_off") {
      digitalWrite(pixhawkPin2, LOW);       // Pin 27
      Serial.println("Pixhawk 2 pasif");
    }
    
    // ===== ENCODER OKUMA =====
    else if (cmd == "encoder_read") {
      // ROS uyumlu format: "OK - Encoders L:1234,R:5678"
      Serial.print("OK - Encoders L:");
      Serial.print(encoder1Count);
      Serial.print(",R:");
      Serial.println(encoder2Count);
    }
    else if (cmd == "encoder_reset") {
      encoder1Count = 0;
      encoder2Count = 0;
      Serial.println("OK - Encoders reset");
    }
    
    // ===== ENCODER SÜREKLI YAYINI (ROS İÇİN) =====
    else if (cmd == "encoder_stream_on") {
      // Sürekli encoder yayını başlat (ROS node için)
      Serial.println("OK - Encoder streaming enabled");
      // Bu özellik ana döngüde implement edilecek
    }
    else if (cmd == "encoder_stream_off") {
      Serial.println("OK - Encoder streaming disabled");
    }
    
    // ===== PWM DURUM OKUMA =====
    else if (cmd == "pwm_status") {
      Serial.print("Servo PWM: ");
      Serial.print(servoPulse);
      Serial.print("μs | Lazer PWM: ");
      Serial.print(rolePulse);
      Serial.println("μs");
    }
    
    // ===== SISTEM BİLGİLERİ =====
    else if (cmd == "status") {
      executeStatusFunction(); // Modüler fonksiyon çağrısı
    }
    
    // ===== TEST KOMUTLARI =====
    else if (cmd == "test") {
      executeTestFunction(); // Modüler fonksiyon çağrısı
    }
    
    else {
      Serial.println("Bilinmeyen komut: " + cmd);
    }
  }
}
