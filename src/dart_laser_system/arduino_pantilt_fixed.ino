/*
  BARLAS Arduino Pan-Tilt Controller - İyileştirilmiş Versiyon
  Servo motor kontrolü ve lazer kontrolü
  Daha güvenilir serial haberleşme ve hareket kontrol
  
  Bağlantılar:
  - Pan Servo  -> Pin 9 (PWM)
  - Tilt Servo -> Pin 10 (PWM)  
  - Laser LED  -> Pin 13 (Digital Out)
  - 5V -> Servo VCC (Ayrı güç kaynağı önerilir)
  - GND -> Ortak GND
  
  Komutlar:
  - TEST -> Bağlantı testi
  - MOVE,pan,tilt -> Her iki servoyu hareket ettir
  - PAN,angle -> Sadece pan servo
  - TILT,angle -> Sadece tilt servo
  - CENTER -> Merkez pozisyon (90,90)
  - LASER,ON/OFF -> Lazer kontrol
  - STATUS -> Mevcut durum
*/

#include <Servo.h>

// Servo nesneleri
Servo panServo;
Servo tiltServo;

// Pin tanımları
const int PAN_PIN = 9;
const int TILT_PIN = 10;
const int LASER_PIN = 13;
const int LED_BUILTIN_PIN = LED_BUILTIN;

// Servo pozisyonları
volatile int currentPan = 90;
volatile int currentTilt = 90;

// Servo limitleri - genişletilmiş
const int PAN_MIN = 0;
const int PAN_MAX = 180;
const int TILT_MIN = 10;
const int TILT_MAX = 170;

// Lazer durumu
volatile bool laserActive = false;

// Serial buffer ve durum
String inputBuffer = "";
bool commandReady = false;
unsigned long lastCommandTime = 0;
unsigned long heartbeatTime = 0;

// Servo hareket parametreleri
const int SERVO_DELAY = 15;  // Servo hareket süre delay (ms)
const int MAX_MOVE_STEP = 5; // Maksimum tek seferde hareket açısı

void setup() {
  // Serial başlat - daha yüksek baud rate
  Serial.begin(9600);
  while (!Serial) {
    ; // Serial hazır olana kadar bekle
  }
  
  // Pin konfigürasyonları
  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  
  // Lazer kapalı başlat
  digitalWrite(LASER_PIN, LOW);
  digitalWrite(LED_BUILTIN_PIN, LOW);
  laserActive = false;
  
  // Servo'ları bağla
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  
  // Servo'lar için güç stabilizasyon delay
  delay(1000);
  
  // Başlangıç pozisyonu - yavaşça merkeze git
  moveToPositionSlow(90, 90);
  
  // Başlatma sinyali
  blinkLED(3);
  
  // Ready mesajı
  Serial.println("BARLAS Arduino Pan-Tilt Ready");
  Serial.println("OK - System Initialized");
  
  heartbeatTime = millis();
  
  // Debug bilgi
  Serial.print("Pan Range: ");
  Serial.print(PAN_MIN);
  Serial.print("-");
  Serial.println(PAN_MAX);
  Serial.print("Tilt Range: ");
  Serial.print(TILT_MIN);
  Serial.print("-");
  Serial.println(TILT_MAX);
}

void loop() {
  // Heartbeat LED - sistem çalıştığını gösterir
  if (millis() - heartbeatTime > 2000) {
    digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
    heartbeatTime = millis();
  }
  
  // Serial veri oku
  readSerialData();
  
  // Komut işleme
  if (commandReady) {
    processCommand();
    commandReady = false;
  }
  
  // Küçük delay - sistem stability
  delay(5);
}

void readSerialData() {
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();
    
    if (incomingByte == '\n' || incomingByte == '\r') {
      if (inputBuffer.length() > 0) {
        commandReady = true;
        lastCommandTime = millis();
        break;
      }
    } else {
      inputBuffer += incomingByte;
      
      // Buffer overflow koruması
      if (inputBuffer.length() > 50) {
        inputBuffer = "";
        Serial.println("ERROR - Buffer overflow");
      }
    }
  }
}

void processCommand() {
  inputBuffer.trim();
  inputBuffer.toUpperCase();
  
  Serial.print("Received: ");
  Serial.println(inputBuffer);
  
  // TEST komutu
  if (inputBuffer == "TEST" || inputBuffer == "PING") {
    Serial.println("OK - Arduino Ready");
    blinkLED(1);
  }
  
  // STATUS komutu
  else if (inputBuffer == "STATUS") {
    Serial.print("OK - Pan:");
    Serial.print(currentPan);
    Serial.print(",Tilt:");
    Serial.print(currentTilt);
    Serial.print(",Laser:");
    Serial.println(laserActive ? "ON" : "OFF");
  }
  
  // CENTER komutu
  else if (inputBuffer == "CENTER") {
    moveToPositionSlow(90, 90);
    Serial.println("OK - Centered");
  }
  
  // MOVE komutu: MOVE,pan,tilt
  else if (inputBuffer.startsWith("MOVE,")) {
    handleMoveCommand();
  }
  
  // PAN komutu: PAN,angle
  else if (inputBuffer.startsWith("PAN,")) {
    handlePanCommand();
  }
  
  // TILT komutu: TILT,angle
  else if (inputBuffer.startsWith("TILT,")) {
    handleTiltCommand();
  }
  
  // LASER komutu: LASER,ON/OFF
  else if (inputBuffer.startsWith("LASER,")) {
    handleLaserCommand();
  }
  
  // LED komutu (alternatif lazer komutu)
  else if (inputBuffer.startsWith("LED,")) {
    handleLaserCommand(); // Aynı fonksiyon
  }
  
  // SERVO komutu (alternatif MOVE)
  else if (inputBuffer.startsWith("SERVO,")) {
    handleMoveCommand(); // Aynı fonksiyon
  }
  
  // Bilinmeyen komut
  else {
    Serial.print("ERROR - Unknown command: ");
    Serial.println(inputBuffer);
  }
  
  // Buffer'ı temizle
  inputBuffer = "";
}

void handleMoveCommand() {
  // Komut formatı: MOVE,90,120
  int firstComma = inputBuffer.indexOf(',');
  int secondComma = inputBuffer.indexOf(',', firstComma + 1);
  
  if (firstComma > 0 && secondComma > 0) {
    int panAngle = inputBuffer.substring(firstComma + 1, secondComma).toInt();
    int tiltAngle = inputBuffer.substring(secondComma + 1).toInt();
    
    if (isValidAngle(panAngle, PAN_MIN, PAN_MAX) && isValidAngle(tiltAngle, TILT_MIN, TILT_MAX)) {
      moveToPositionSlow(panAngle, tiltAngle);
      Serial.print("OK - Moved to Pan:");
      Serial.print(currentPan);
      Serial.print(",Tilt:");
      Serial.println(currentTilt);
    } else {
      Serial.print("ERROR - Invalid angles: Pan:");
      Serial.print(panAngle);
      Serial.print(",Tilt:");
      Serial.println(tiltAngle);
    }
  } else {
    Serial.println("ERROR - Invalid MOVE format (use: MOVE,pan,tilt)");
  }
}

void handlePanCommand() {
  // Komut formatı: PAN,90
  int commaIndex = inputBuffer.indexOf(',');
  
  if (commaIndex > 0) {
    int panAngle = inputBuffer.substring(commaIndex + 1).toInt();
    
    if (isValidAngle(panAngle, PAN_MIN, PAN_MAX)) {
      moveToPositionSlow(panAngle, currentTilt);
      Serial.print("OK - Pan moved to ");
      Serial.println(currentPan);
    } else {
      Serial.print("ERROR - Invalid pan angle: ");
      Serial.println(panAngle);
    }
  } else {
    Serial.println("ERROR - Invalid PAN format (use: PAN,angle)");
  }
}

void handleTiltCommand() {
  // Komut formatı: TILT,120
  int commaIndex = inputBuffer.indexOf(',');
  
  if (commaIndex > 0) {
    int tiltAngle = inputBuffer.substring(commaIndex + 1).toInt();
    
    if (isValidAngle(tiltAngle, TILT_MIN, TILT_MAX)) {
      moveToPositionSlow(currentPan, tiltAngle);
      Serial.print("OK - Tilt moved to ");
      Serial.println(currentTilt);
    } else {
      Serial.print("ERROR - Invalid tilt angle: ");
      Serial.println(tiltAngle);
    }
  } else {
    Serial.println("ERROR - Invalid TILT format (use: TILT,angle)");
  }
}

void handleLaserCommand() {
  // Komut formatı: LASER,ON veya LASER,OFF
  int commaIndex = inputBuffer.indexOf(',');
  
  if (commaIndex > 0) {
    String laserState = inputBuffer.substring(commaIndex + 1);
    
    if (laserState == "ON" || laserState == "1") {
      digitalWrite(LASER_PIN, HIGH);
      laserActive = true;
      Serial.println("OK - Laser ON");
      blinkLED(2);
    } else if (laserState == "OFF" || laserState == "0") {
      digitalWrite(LASER_PIN, LOW);
      laserActive = false;
      Serial.println("OK - Laser OFF");
    } else {
      Serial.print("ERROR - Invalid laser state: ");
      Serial.println(laserState);
    }
  } else {
    Serial.println("ERROR - Invalid LASER format (use: LASER,ON/OFF)");
  }
}

bool isValidAngle(int angle, int minVal, int maxVal) {
  return (angle >= minVal && angle <= maxVal);
}

void moveToPositionSlow(int targetPan, int targetTilt) {
  // Hedef açıları sınırla
  targetPan = constrain(targetPan, PAN_MIN, PAN_MAX);
  targetTilt = constrain(targetTilt, TILT_MIN, TILT_MAX);
  
  Serial.print("Moving to Pan:");
  Serial.print(targetPan);
  Serial.print(", Tilt:");
  Serial.println(targetTilt);
  
  // Smooth hareket - adım adım git
  while (currentPan != targetPan || currentTilt != targetTilt) {
    // Pan hareketi
    if (currentPan < targetPan) {
      currentPan = min(currentPan + MAX_MOVE_STEP, targetPan);
    } else if (currentPan > targetPan) {
      currentPan = max(currentPan - MAX_MOVE_STEP, targetPan);
    }
    
    // Tilt hareketi
    if (currentTilt < targetTilt) {
      currentTilt = min(currentTilt + MAX_MOVE_STEP, targetTilt);
    } else if (currentTilt > targetTilt) {
      currentTilt = max(currentTilt - MAX_MOVE_STEP, targetTilt);
    }
    
    // Servo pozisyonlarını güncelle
    panServo.write(currentPan);
    tiltServo.write(currentTilt);
    
    delay(SERVO_DELAY);
    
    // Serial buffer'ı kontrol et (kesintisiz hareket için)
    if (Serial.available() > 0) {
      readSerialData();
    }
  }
  
  // Son pozisyon kontrolü
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
  
  Serial.print("Movement completed. Final position: Pan:");
  Serial.print(currentPan);
  Serial.print(", Tilt:");
  Serial.println(currentTilt);
}

void moveToPositionFast(int targetPan, int targetTilt) {
  // Hızlı hareket - direkt pozisyon
  targetPan = constrain(targetPan, PAN_MIN, PAN_MAX);
  targetTilt = constrain(targetTilt, TILT_MIN, TILT_MAX);
  
  currentPan = targetPan;
  currentTilt = targetTilt;
  
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
  
  delay(500); // Servo'ların pozisyona gitmesi için bekle
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN_PIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN_PIN, LOW);
    delay(200);
  }
}

// Watchdog fonksiyonu - sistem donma koruması
void watchdog() {
  if (millis() - lastCommandTime > 30000) { // 30 saniye komut gelmezse
    // Sistem reset - merkez pozisyona dön, lazeri kapat
    moveToPositionFast(90, 90);
    digitalWrite(LASER_PIN, LOW);
    laserActive = false;
    Serial.println("WATCHDOG - System reset to safe position");
    lastCommandTime = millis();
  }
}
