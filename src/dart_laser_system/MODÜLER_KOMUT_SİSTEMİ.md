# BARLAS MODÜLER KOMUT SİSTEMİ
# Arduino Mega 2560 - Pin Kontrol ve Char Komut Sistemi
# Tarih: 3 Eylül 2025

## 🎮 TEK KARAKTER KOMUTLAR (Hızlı Erişim)

| Karakter | İşlev | Açıklama |
|----------|-------|----------|
| `b` | Brake | Fren uygula (Pin 9-10) |
| `r` | Release | Fren bırak (Pin 9-10) |
| `l` | Laser On | Lazer röle aç (Pin 13 HIGH) |
| `o` | Laser Off | Lazer röle kapat (Pin 13 LOW) |
| `c` | Activate | Pixhawk manuel aktif (Pin 23) |
| `d` | Deactivate | Pixhawk pasif (Pin 23) |
| `f` | Far On | Far aç (Pin 6 HIGH) |
| `g` | Far Off | Far kapat (Pin 6 LOW) |
| `s` | Status | Sistem durumu görüntüle |
| `t` | Test | Bağlantı ve pin testi |

### Kullanım Örnekleri:
```
b        # Fren uygula
l        # Lazer aç
c        # Pixhawk aktif et
s        # Status göster
```

---

## 🤖 OTOMATIK PIN KONTROL SİSTEMİ

### Pin 23 (Pixhawk Manuel) Durumu:
```cpp
// Pin 23 HIGH algılandığında:
executePixhawkActivateFunction();  // char 'c' işlevi

// Pin 23 LOW algılandığında:  
executePixhawkDeactivateFunction(); // char 'd' işlevi
```

### PWM Sinyal Kontrolü:
```cpp
// Pin 2 PWM > 90 algılandığında:
executeBrakeFunction();      // char 'b' işlevi

// Pin 2 PWM < 90 algılandığında:
executeReleaseFunction();    // char 'r' işlevi

// Pin 3 PWM > 90 algılandığında:
executeLaserOnFunction();    // char 'l' işlevi

// Pin 3 PWM < 90 algılandığında:
executeLaserOffFunction();   // char 'o' işlevi
```

---

## 🔧 MODÜLER FONKSİYON SİSTEMİ

### Fren Sistemi:
```cpp
void executeBrakeFunction() {
  frenServo1.write(servoOffset1);  // Pin 9
  frenServo2.write(servoOffset2);  // Pin 10
  Serial.println("[AUTO] Fren uygulandı");
}

void executeReleaseFunction() {
  frenServo1.write(90);            // Pin 9-10 nötr
  frenServo2.write(90);
  Serial.println("[AUTO] Fren bırakıldı");
}
```

### Lazer Sistemi:
```cpp
void executeLaserOnFunction() {
  digitalWrite(lazerRolePin, HIGH);  // Pin 13
  Serial.println("[AUTO] Lazer röle açıldı");
}

void executeLaserOffFunction() {
  digitalWrite(lazerRolePin, LOW);   // Pin 13
  Serial.println("[AUTO] Lazer röle kapatıldı");
}
```

### Pixhawk Kontrol:
```cpp
void executePixhawkActivateFunction() {
  pixhawkActive = true;
  controlMode = "PIXHAWK";
  enableAutoPilot();               // Pin 26-27 HIGH
  Serial.println("[PIN23] Pixhawk modu aktif!");
}

void executePixhawkDeactivateFunction() {
  pixhawkActive = false; 
  controlMode = "SERIAL";
  disableAutoPilot();              // Pin 26-27 LOW
  Serial.println("[PIN23] Manuel mod başladı");
}
```

---

## ⚡ ÇALIŞMA MANTIĞI

### 1. Pin Durumu Sürekli Kontrol:
```cpp
void loop() {
  checkPixhawkPinStates();  // Pin 23 sürekli kontrol
  // PWM sinyalleri sürekli kontrol
  // Serial komutlar işlenir
}
```

### 2. Pin 23 Debouncing:
```cpp
void checkPixhawkPinStates() {
  static bool lastPixhawkState = false;
  static unsigned long lastCheckTime = 0;
  
  // Her 100ms kontrol et
  if (millis() - lastCheckTime > 100) {
    bool currentState = digitalRead(pixhawkManuelPin);
    
    if (currentState && !lastPixhawkState) {
      // Pin yeni HIGH oldu
      executePixhawkActivateFunction();
    }
  }
}
```

### 3. Modüler Çağrı Sistemi:
- **PWM Sinyali** → Otomatik fonksiyon çağrısı
- **Pin 23 Durumu** → Otomatik fonksiyon çağrısı  
- **Char Komut** → Manuel fonksiyon çağrısı
- **String Komut** → Manuel fonksiyon çağrısı (eski uyumluluk)

---

## 📊 SİSTEM ÇIKTILARI

### Otomatik İşlem Çıktıları:
```
[AUTO] Fren uygulandı
[AUTO] Lazer röle açıldı
[PIN23] Pixhawk modu aktif!
[AUTOPILOT] X-Y kontrol aktif
```

### Manuel Komut Çıktıları:
```
[CHAR] Far açıldı  
[CMD] Far açıldı
```

### Status Çıktısı:
```
=== BARLAS SİSTEM DURUM (GÜNCEL PIN) ===
Encoder1 (Pin18-19): 1234
Encoder2 (Pin20-21): 5678
Servo PWM (Pin2): 1500μs
Lazer PWM (Pin3): 1000μs
Far (Pin6): OFF
Pan-Tilt (Pin7-8): Pan=90° Tilt=90°
Fren (Pin9-10): 90° / 90°
Lazer Röle (Pin13): OFF
Pixhawk Manuel (Pin23): OFF
Pixhawk X-Y (Pin26-27): OFF / OFF
Kontrol Modu: SERIAL
========================================
```

---

## 🚀 AVANTAJLAR

### ✅ Modüler Yapı:
- Tek fonksiyon = Tek işlev
- Kolay hata ayıklama
- Bakım kolaylığı

### ✅ Çoklu Erişim:
- Char komut: `l` 
- String komut: `lazer_on`
- PWM sinyal: Otomatik
- Pin durumu: Otomatik

### ✅ Temiz Kod:
- Tekrar eden kodlar yok
- Switch-case yapısı
- İşlev odaklı fonksiyonlar

### ✅ Gerçek Zamanlı:
- Pin durumları sürekli izleniyor
- PWM sinyalleri otomatik işleniyor
- Debouncing sistemi

Bu sistem sayede **Pin 23 HIGH olduğunda otomatik olarak `char c` işlevi çalışır** ve sistem çok daha temiz ve yönetilebilir hale gelir! 🎯
