# 🎯 BARLAS Arduino Dart Targeting - Donanım Kurulum Rehberi

## 🔧 Donanım Bağlantıları

### 📹 1. Kamera Bağlantısı
```
USB Kamera → Laptop USB Port
veya
Dahili kamera kullan
```

### 🤖 2. Arduino Bağlantısı  
```
Arduino Uno → Laptop USB Port (COM3, COM4, COM5...)
```

### ⚡ 3. Arduino Donanım Bağlantıları
```cpp
// Pin Bağlantıları:
Pan Servo    → Pin 9    (Sarı kablo)
Tilt Servo   → Pin 10   (Sarı kablo)  
Laser Diode  → Pin 13   (+ dirençle)
GND          → Arduino GND (Siyah kablolar)
VCC (5V)     → Arduino 5V (Kırmızı kablolar)

// Önemli: Servo'lar için ayrı güç kaynağı önerilir!
```

## 🚀 Sistem Başlatma

### 📹 Adım 1: Kamera Testi
```bash
# Mevcut kameraları test et
python camera_test.py

# Çıktı örneği:
✅ Kamera 0: 640x480 - Dahili
✅ Kamera 1: 1920x1080 - USB
```

### 🤖 Adım 2: Arduino Firmware Yükleme
1. **Arduino IDE** aç
2. **arduino_pantilt.ino** dosyasını aç  
3. **Arduino Uno** seç (Tools → Board)
4. **COM Port** seç (Tools → Port)
5. **Upload** et (Ctrl+U)

### 🎯 Adım 3: Sistem Çalıştırma

**USB Kamera ile:**
```bash
python yolo_arduino_dart_system.py --camera 1
```

**Dahili kamera ile:**
```bash
python yolo_arduino_dart_system.py --camera 0
```

**Otomatik kamera tespiti:**
```bash
python yolo_arduino_dart_system.py
```

## 🎮 Kontroller (Hibrit Sistem: Otomatik + Manuel)

**🤖 OTOMATIK MOD:** Sistem sürekli dart arıyor, bulunca 2 saniye sonra otomatik hedefliyor  
**🎮 MANUEL MOD:** Sen istediğin zaman müdahale edebilirsin (WASD tuşları)

| Tuş | Fonksiyon | Mod |
|-----|-----------|-----|
| `q` | Sistemden çık | Sistem |
| `space` | Manuel lazer açma/kapama | 🎮 Manuel |
| `c` | Merkez pozisyon | 🎮 Manuel |
| `w` | Yukarı (Tilt -5°) | 🎮 Manuel |
| `s` | Aşağı (Tilt +5°) | 🎮 Manuel |
| `a` | Sol (Pan -5°) | 🎮 Manuel |
| `d` | Sağ (Pan +5°) | 🎮 Manuel |
| `+` | Güven eşiği artır | Ayar |
| `-` | Güven eşiği azalt | Ayar |

**💡 ÖNEMLİ:** Otomatik sistem sürekli çalışır, sen istediğin zaman manuel kontrol alabilirsin!

## 🔍 Sistem Çıktısı

**Arduino Bağlantısı:**
```
✅ Arduino Controller modülü yüklendi!
🔍 Arduino bağlantısı deneniyor: COM3
✅ Gerçek Arduino bağlandı: COM3
```

**Kamera Tespiti:**
```
📹 Kamera: 1 (USB)
✅ Kamera: 1920x1080
```

**Dart Tespiti:**
```
🎯 Dart merkezi bulundu: (245, 378), r=34
🔥 DART KİLİTLENDİ! Hough Circle Merkez: (245, 378)
🚀 Arduino'ya hedefleme komutu gönderiliyor...
🎯 HEDEF: Piksel (245, 378) -> Servo (82°, 95°)
🔄 SERVO KOMUTU: MOVE,82,95
🔴 ARDUINO KOMUTU: LASER,ON
```

## 🎯 Kalibrasyon

Eğer lazer tam hedefe gelmiyorsa:

```python
# yolo_arduino_dart_system.py içinde:
controller.calibrate_offset(pixel_offset_x=10, pixel_offset_y=-5)
```

## ❌ Sorun Giderme

**Arduino bulunamıyor:**
- COM port kontrol edin (Device Manager)
- Arduino IDE'de bağlantıyı test edin  
- Firmware yüklenmiş mi kontrol edin

**Kamera açılmıyor:**
- `python camera_test.py` çalıştırın
- Farklı kamera index deneyin (0, 1, 2...)
- Başka uygulama kamera kullanıyor mu kontrol edin

**YOLO modülü bulunamıyor:**
- `dart_recognize` klasörü var mı kontrol edin
- Python path ayarları kontrol edin

## 🎯 Test Süreci

1. **Kamera testi** ✅
2. **Arduino firmware upload** ✅  
3. **Sistem başlatma** ✅
4. **Dart gösterme** → Otomatik tespit
5. **2 saniye bekleme** → Hedef kilitleme
6. **Servo hareketi** → Lazer açılması

**Sistem hazır! 🚀**
