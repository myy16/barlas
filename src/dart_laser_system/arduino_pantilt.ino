/*
  BARLAS Arduino Pan-Tilt Controller
  Servo motor kontrolü ve lazer kontrolü
  Serial komut üzerinden Python ile haberleşme
  
  Bağlantılar:
  - Pan Servo  -> Pin 9
  - Tilt Servo -> Pin 10  
  - Laser      -> Pin 13
  - GND        -> Arduino GND
  - VCC        -> 5V (Servo'lar için ayrı güç önerilir)
*/

#include <Servo.h>

// Servo nesneleri
Servo panServo;
Servo tiltServo;

// Pin tanımları
const int PAN_PIN = 9;
const int TILT_PIN = 10;
const int LASER_PIN = 13;

// Servo pozisyonları
int panPosition = 90;    // Başlangıç merkez
int tiltPosition = 90;   // Başlangıç merkez

// Servo limitleri
const int PAN_MIN = 10;
const int PAN_MAX = 170;
const int TILT_MIN = 30;
const int TILT_MAX = 150;

// Lazer durumu
bool laserActive = false;

// Serial buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  // Serial başlat
  Serial.begin(9600);
  
  // Servo'ları pin'lere bağla
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  
  // Lazer pin'ini çıkış yap
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  
  // Başlangıç pozisyonuna git
  panServo.write(panPosition);
  tiltServo.write(tiltPosition);
  
  // Kurulum tamamlandı
  Serial.println("BARLAS Arduino Pan-Tilt Ready");
  delay(1000);
}

void loop() {
  // Serial veri kontrolü
  serialEvent();
  
  // Komut geldi mi?
  if (stringComplete) {
    processCommand(inputString);
    
    // Buffer'ı temizle
    inputString = "";
    stringComplete = false;
  }
  
  delay(10);
}

// Serial olay işleyici
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Komut işleyici
void processCommand(String command) {
  command.trim();
  
  // TEST komutu
  if (command == "TEST") {
    Serial.println("OK - Arduino Ready");
    return;
  }
  
  // MOVE komutu: MOVE,pan_angle,tilt_angle
  if (command.startsWith("MOVE,")) {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    
    if (firstComma > 0 && secondComma > 0) {
      int newPan = command.substring(firstComma + 1, secondComma).toInt();
      int newTilt = command.substring(secondComma + 1).toInt();
      
      moveServos(newPan, newTilt);
      Serial.println("OK - Moved");
    } else {
      Serial.println("ERROR - Invalid MOVE format");
    }
    return;
  }
  
  // LASER komutu: LASER,ON veya LASER,OFF
  if (command.startsWith("LASER,")) {
    String laserCommand = command.substring(6);
    
    if (laserCommand == "ON") {
      digitalWrite(LASER_PIN, HIGH);
      laserActive = true;
      Serial.println("OK - Laser ON");
    } else if (laserCommand == "OFF") {
      digitalWrite(LASER_PIN, LOW);
      laserActive = false;
      Serial.println("OK - Laser OFF");
    } else {
      Serial.println("ERROR - Invalid LASER command");
    }
    return;
  }
  
  // PAN komutu: PAN,angle
  if (command.startsWith("PAN,")) {
    int angle = command.substring(4).toInt();
    moveServos(angle, tiltPosition);
    Serial.println("OK - Pan moved");
    return;
  }
  
  // TILT komutu: TILT,angle
  if (command.startsWith("TILT,")) {
    int angle = command.substring(5).toInt();
    moveServos(panPosition, angle);
    Serial.println("OK - Tilt moved");
    return;
  }
  
  // CENTER komutu
  if (command == "CENTER") {
    moveServos(90, 90);
    Serial.println("OK - Centered");
    return;
  }
  
  // STATUS komutu
  if (command == "STATUS") {
    Serial.print("OK - Pan:");
    Serial.print(panPosition);
    Serial.print(",Tilt:");
    Serial.print(tiltPosition);
    Serial.print(",Laser:");
    Serial.println(laserActive ? "ON" : "OFF");
    return;
  }
  
  // Bilinmeyen komut
  Serial.println("ERROR - Unknown command: " + command);
}

// Servo hareket fonksiyonu
void moveServos(int newPan, int newTilt) {
  // Sınırları kontrol et
  newPan = constrain(newPan, PAN_MIN, PAN_MAX);
  newTilt = constrain(newTilt, TILT_MIN, TILT_MAX);
  
  // Pozisyonları güncelle
  panPosition = newPan;
  tiltPosition = newTilt;
  
  // Servo'ları hareket ettir
  panServo.write(panPosition);
  tiltServo.write(tiltPosition);
  
  // Servo'ların hareket etmesini bekle
  delay(200);
}

// Smooth hareket fonksiyonu (isteğe bağlı)
void smoothMove(int targetPan, int targetTilt) {
  int stepDelay = 20;
  
  while (panPosition != targetPan || tiltPosition != targetTilt) {
    // Pan hareketi
    if (panPosition < targetPan) {
      panPosition++;
    } else if (panPosition > targetPan) {
      panPosition--;
    }
    
    // Tilt hareketi
    if (tiltPosition < targetTilt) {
      tiltPosition++;
    } else if (tiltPosition > targetTilt) {
      tiltPosition--;
    }
    
    // Servo'ları güncelle
    panServo.write(panPosition);
    tiltServo.write(tiltPosition);
    
    delay(stepDelay);
  }
}

// Debug bilgi yazdır
void printStatus() {
  Serial.print("Pan: ");
  Serial.print(panPosition);
  Serial.print("°, Tilt: ");
  Serial.print(tiltPosition);
  Serial.print("°, Laser: ");
  Serial.println(laserActive ? "ON" : "OFF");
}
