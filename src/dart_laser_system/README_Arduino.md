# BARLAS Arduino Dart Laser System

Arduino tabanlı Pan-Tilt servo kontrolü ile dart hedefleme sistemi.

## 🔧 Donanım Gereksinimleri

### Arduino Bağlantıları
```
Arduino Uno:
- Pan Servo (SG90)    -> Pin 9
- Tilt Servo (SG90)   -> Pin 10  
- Laser Diode         -> Pin 13
- Servo VCC           -> 5V (Ayrı güç önerilir)
- Servo GND           -> GND
- Laser GND           -> GND
```

### Gerekli Malzemeler
- Arduino Uno R3
- 2x SG90 Micro Servo Motor
- 1x Laser Diode (5V)
- Pan-Tilt Bracket
- USB Kamera (Webcam)
- Jumper Kablolar
- Breadboard (opsiyonel)

## 📦 Python Gereksinimleri

```bash
pip install opencv-python numpy pyserial
```

## 🚀 Kurulum ve Kullanım

### 1. Arduino Kurulumu
1. `arduino_pantilt.ino` dosyasını Arduino IDE ile açın
2. Arduino Uno'ya yükleyin
3. Serial Monitor'da "BARLAS Arduino Pan-Tilt Ready" mesajını bekleyin

### 2. Python Sistemi Test
```bash
cd dart_laser_system
python arduino_controller.py
```

### 3. Dart Hedefleme Sistemi
```bash
python arduino_dart_system.py
```

## 🎮 Kontroller

### Hedefleme Sistemi
- `q` - Sistemden çıkış
- `space` - Manuel lazer açma/kapama
- `c` - Servo'ları merkez pozisyona getir
- `w/s` - Tilt servo yukarı/aşağı
- `a/d` - Pan servo sol/sağ

### Kalibrasyon Modu
- `w/a/s/d` - Servo hareket
- `space` - Lazer açma/kapama
- `+/-` - X offset ayarı
- `u/j` - Y offset ayarı
- `c` - Merkez pozisyon
- `q` - Çıkış

## 📊 Sistem Parametreleri

```python
# Hedefleme ayarları
confidence_threshold = 0.5    # Dart tespit güven eşiği
lock_time = 1.5              # Hedefe kilitlenme süresi (saniye)
laser_duration = 3.0         # Lazer açık kalma süresi
stability_threshold = 25     # Hedef kararlılık eşiği (piksel)
```

## 🔍 Sorun Giderme

### Arduino Bağlantı Sorunları
1. **Port Hatası**: Device Manager'dan doğru COM port'unu kontrol edin
2. **Driver Eksik**: Arduino IDE'dan driver yükleyin
3. **Güç Sorunu**: Servo'lar için ayrı 5V güç kaynağı kullanın

### Kamera Sorunları
1. **Kamera Bulunamadı**: `camera_index=0` değerini 1,2,3 deneyin
2. **Düşük FPS**: Kamera çözünürlüğünü düşürün
3. **Webcam Çakışması**: Diğer kamera uygulamalarını kapatın

### YOLO Tespit Sorunları
1. **Dart Bulunamıyor**: `confidence_threshold` değerini düşürün
2. **Yanlış Tespit**: Aydınlatmayı iyileştirin
3. **Çok Hassas**: `stability_threshold` değerini artırın

## 📋 Arduino Komut Protokolü

### Desteklenen Komutlar
- `TEST` - Arduino hazır mı kontrol
- `MOVE,pan,tilt` - Servo pozisyon (örn: MOVE,90,90)
- `LASER,ON/OFF` - Lazer kontrolü
- `PAN,angle` - Sadece pan servo
- `TILT,angle` - Sadece tilt servo
- `CENTER` - Merkez pozisyon (90,90)
- `STATUS` - Mevcut pozisyon bilgisi

### Yanıt Formatı
- `OK` - Komut başarılı
- `ERROR - mesaj` - Komut hatası

## ⚙️ Kalibrasyon Rehberi

1. **Arduino Kontrolü Test**:
   ```bash
   python arduino_controller.py
   ```

2. **Kamera-Lazer Hizalama**:
   - Kamera görüntüsünde merkez çizgiyi göreceksiniz
   - Lazer'i açın ve merkez noktasına hizalayın
   - Offset değerlerini ayarlayın

3. **Dart Tespit Test**:
   - Kameraya dart gösterin
   - Tespit edilen dart'ların etrafında yeşil kutu görünmelidir
   - Güven eşiğini ihtiyaca göre ayarlayın

## 🎯 Hedefleme Algoritması

1. **Dart Tespiti**: YOLO ile kameradan dart tespit
2. **Kararlılık Testi**: Hedefin sabit olup olmadığını kontrol
3. **Kilitlenme**: Belirtilen süre boyunca hedef stabil kalırsa kilitle
4. **Lazer Ateş**: Arduino'ya koordinat gönder, servo'ları hareket ettir
5. **Lazer Kontrolü**: Belirtilen süre boyunca lazer açık tut

## 🔧 Donanım Notları

- **Servo Güç**: SG90 servo'lar 5V/1A güç gerektirir
- **Lazer Güvenlik**: Lazer gözle temas etmemelidir
- **Pan-Tilt Montaj**: Servo'lar düzgün hizalanmalıdır
- **Kablo Uzunluğu**: USB kablosu max 3m olmalıdır

## 📝 Sistem Durumu

```
✅ Arduino Controller: Test edildi, çalışır durumda
✅ Dart Detector: YOLO entegrasyonu hazır  
✅ Targeting System: Tam otomatik hedefleme
✅ Manual Control: WASD kontrolleri aktif
✅ Calibration: Kamera-lazer hizalama sistemi
```

## 🎮 Gelişmiş Özellikler

- **Smooth Servo Movement**: Servo'lar yumuşak hareket eder
- **Real-time Targeting**: 30 FPS dart tespit ve hedefleme
- **Automatic Calibration**: Kamera-lazer otomatik hizalama
- **Multiple Dart Handling**: Birden fazla dart arasında en iyisini seç
- **Safety Timeouts**: Lazer otomatik kapanma koruması

## 🐛 Bilinen Sorunlar

1. **Servo Jitter**: Güç kaynağı yetersizliği
2. **Lazer Drift**: Kalibrasyon gerekli
3. **Detection Lag**: Bilgisayar performansı düşük

## 🔄 Güncellemeler

- v1.0: Temel Arduino kontrolü
- v1.1: YOLO dart tespiti entegrasyonu  
- v1.2: Otomatik hedefleme sistemi
- v1.3: Kalibrasyon ve manuel kontrol

---
**BARLAS Team - Arduino Dart Laser Targeting System**
