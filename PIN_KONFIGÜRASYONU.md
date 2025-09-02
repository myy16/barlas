# ⚡ BARLAS ARDUINO PIN KONFIGÜRASYONU VE BAĞLANTI ŞEMASI

## 🔌 KRİTİK NOKTA: Arduino otomatik pin tespiti YAPMAZ!

Arduino kodunda **SABİT PIN ATAMALARI** var. Bu pinler değiştirilmezse sistem çalışmaz!

# ⚡ BARLAS ARDUINO - GERÇEK PIN KONFIGÜRASYONU

## 🔌 GERÇEK DONANIM PIN ATLASI

Bu pin atamaları sizin **GERÇEK donanım konfigürasyonunuza** göre ayarlanmıştır!

### 🎯 PAN-TILT VE SERVO SİSTEMİ
```cpp
Servo panServo   → Pin 6   (PWM) 
Servo tiltServo  → Pin 7   (PWM)
Servo servo3     → Pin 8   (PWM) - Ek servo
Servo servo4     → Pin 9   (PWM) - Ek servo
```
**5V Regülatör:** Bu servo'lar 5V regülatörden beslenir

### 🔥 LAZER SİSTEMİ
```cpp
int laserPin     → Pin 13  (Digital Output)
```

### � RÖLE KONTROLLARI
```cpp
int headlightPin → Pin 22  (Digital Output - Röle 1)
int relay2Pin    → Pin 23  (Digital Output - Röle 2)
```
**5V Regülatör:** Röle kontrol devreleri 5V regülatörden beslenir

### 📊 ENCODER SİSTEMİ (Interrupt Pinleri)
```cpp
int encoder1PinA → Pin 2   (Interrupt 0 - Sol Encoder A)
int encoder1PinB → Pin 3   (Interrupt 1 - Sol Encoder B)
int encoder2PinA → Pin 18  (Interrupt 5 - Sağ Encoder A)
int encoder2PinB → Pin 19  (Interrupt 4 - Sağ Encoder B)
```
**Arduino:** Bu encoder'lar doğrudan Arduino'ya bağlı

### ⚡ GÜÇ YÖNETİMİ
```cpp
int regulator5VPin → Pin 10  (Digital Output - 5V Regülatör Kontrol)
```

## � YENİ KOMUTLAR

### Arduino Serial Monitor'de Test Komutları:
```cpp
// Temel testler
TEST                     → "OK"
STATUS                   → Tüm pin durumları

// Pan-Tilt ve servo kontrolleri  
MOVE,90,90              → Pan-Tilt merkez (Pin 6,7)
SERVO3,45               → Servo 3 kontrol (Pin 8)
SERVO4,135              → Servo 4 kontrol (Pin 9)

// Lazer kontrolü
LASER,ON                → Lazer aç (Pin 13)
LASER,OFF               → Lazer kapat (Pin 13)

// Röle kontrolleri
HEADLIGHT_ON            → Röle 1 aç (Pin 22) 
HEADLIGHT_OFF           → Röle 1 kapat (Pin 22)
RELAY2_ON               → Röle 2 aç (Pin 23)
RELAY2_OFF              → Röle 2 kapat (Pin 23)

// Güç yönetimi
5V_ON                   → 5V regülatör aç (Pin 10)
5V_OFF                  → 5V regülatör kapat (Pin 10)

// Encoder okuma
GET_ENCODERS            → Encoder değerleri (Pin 2,3,18,19)
RESET_ENCODERS          → Encoder sıfırla
```

## ⚠️ HAYATI ÖNEM: DOĞRU BAĞLANTI


### ✅ GERÇEK DURUM:
- **HER PIN SABİT KODLANMIŞ!**
- Yanlış pin bağlantısı = sistem çalışmaz
- Interrupt pinleri özel (2,3,18,19,20,21)

## 🔧 PIN KONFIGÜRASYON DOSYASI

Arduino kodunda pin değişikliği yapmak için:

```cpp
// ===== PIN TANIMLAMALARI =====
// VAROLAN SİSTEM PİNLERİ - DEĞİŞTİRMEYİN!
int laserPin = 13;
int R_EN = 2;
int L_EN = 3;
int RPWM = 4;
int LPWM = 5;
// ... vs

// EĞER PİN DEĞİŞTİRMEK İSTİYORSANIZ:
// 1. Burada değiştirin
// 2. Donanım bağlantısını değiştirin
// 3. Kodu tekrar yükleyin
```

## 📋 BAĞLANTI KONTROL LİSTESİ

Arkadaşınız için **MUTLAKA KONTROL ETMESİ** gerekenler:

### 1. PAN-TILT BAĞLANTISI
```
✅ Pan Servo  → Arduino Pin 9
✅ Tilt Servo → Arduino Pin 10
✅ Lazer      → Arduino Pin 13
✅ Servo VCC  → 5V (harici güç önerilir)
✅ Servo GND  → GND
```

### 2. MOTOR SÜRÜCÜ BAĞLANTISI (BTS7960)
```
✅ R_EN  → Arduino Pin 2
✅ L_EN  → Arduino Pin 3
✅ RPWM  → Arduino Pin 4
✅ LPWM  → Arduino Pin 5
✅ R_IS  → Arduino Pin A0
✅ L_IS  → Arduino Pin A1
✅ VCC   → Motor gücü (12V-24V)
✅ GND   → Ortak toprak
```

### 3. FREN VE FAR BAĞLANTISI
```
✅ Fren Servo → Arduino Pin 6
✅ Far Röle   → Arduino Pin 7
✅ GND        → Ortak toprak
```

### 4. ENCODER BAĞLANTISI (EN ÖNEMLİ!)
```
✅ Sol Encoder A  → Arduino Pin 21 (Interrupt)
✅ Sol Encoder B  → Arduino Pin 20 (Interrupt)
✅ Sağ Encoder A → Arduino Pin 19 (Interrupt)
✅ Sağ Encoder B → Arduino Pin 18 (Interrupt)
✅ Encoder VCC   → 5V
✅ Encoder GND   → GND
```

## 🧪 PIN TEST KOMUTLARI

### Arduino Serial Monitor'de test:
```cpp
// Pin durumları
STATUS                    → Tüm pin durumlarını gösterir

// Tekil pin testleri
LASER,ON                 → Pin 13 HIGH
LASER,OFF                → Pin 13 LOW
HEADLIGHT_ON             → Pin 7 HIGH  
BRAKE_ON                 → Pin 6 servo 90°
MOTOR_FORWARD,100        → Pin 4 PWM 100

// Encoder okuma
GET_ENCODERS             → Pin 18,19,20,21 interrupt sayıları
```

### Python'da pin test:
```python
from arduino_controller_fixed import BarlasVehicleController

controller = BarlasVehicleController(port="COM3")
controller.connect()

# Her sistemi tek tek test et
controller.enable_laser()      # Pin 13 test
controller.headlight_on()      # Pin 7 test  
controller.brake_on()          # Pin 6 test
controller.move_forward(50)    # Pin 2,3,4,5 test
encoders = controller.read_encoders()  # Pin 18,19,20,21 test

controller.disconnect()
```

## 🔧 PİN DEĞİŞİKLİĞİ NASIL YAPILIR?

Eğer donanım farklı pinlerde bağlıysa:

### ADIM 1: Arduino kodunu değiştir
```cpp
// arduino_pantilt_fixed.ino dosyasında
// ===== PIN TANIMLAMALARI =====
int laserPin = 12;        // 13'ten 12'ye değiştirdik
int headlightPin = 8;     // 7'den 8'e değiştirdik
// ... diğer pinler
```

### ADIM 2: Kodu tekrar yükle
```
Arduino IDE → Upload (Ctrl+U)
```

### ADIM 3: Python tarafını güncelle (gerekirse)
```python
# Genelde Python tarafında değişiklik gerekmez
# Çünkü Python komut gönderir, Arduino pinleri kontrol eder
```

## ⚡ INTERRUPT PİNLERİ (ÇOK ÖNEMLİ!)

### Arduino Mega Interrupt Pinleri:
```
Pin 2  → Interrupt 0
Pin 3  → Interrupt 1  
Pin 18 → Interrupt 5
Pin 19 → Interrupt 4
Pin 20 → Interrupt 3
Pin 21 → Interrupt 2
```

### Arduino Uno Interrupt Pinleri:
```
Pin 2 → Interrupt 0
Pin 3 → Interrupt 1
```

**⚠️ DİKKAT:** Encoder'lar sadece interrupt pinlerinde çalışır!

## 🚨 YAYGÍN HATALAR

### 1. Yanlış Pin Bağlantısı
```
❌ Motor sürücü Pin 6'ya bağlı ama kod Pin 4 bekliyor
✅ Motor sürücü Pin 4'e bağlı ve kod Pin 4 kullanıyor
```

### 2. Interrupt Pin Hatası
```
❌ Encoder Pin 12'ye bağlı (interrupt değil)
✅ Encoder Pin 21'e bağlı (interrupt pin)
```

### 3. Güç Sorunları
```
❌ Servo'lar Arduino 5V'den besleniyor (yetersiz)
✅ Servo'lar harici güçten besleniyor
```

## 📞 SORUN GİDERME

### Pin çalışmıyor?
1. **Bağlantı kontrolü:** Multimetre ile pin voltajını ölç
2. **Kod kontrolü:** Serial Monitor'de pin durumunu kontrol et
3. **Güç kontrolü:** Yeterli akım var mı kontrol et

### Encoder okuma yok?
1. **Interrupt pin kontrolü:** Pin 18,19,20,21 kullanılıyor mu?
2. **Bağlantı kontrolü:** A/B kanalları doğru mu?
3. **Güç kontrolü:** Encoder'a 5V geliyor mu?

Bu bilgilerle arkadaşınız pin bağlantılarını doğru yapabilir! 🔌⚡
