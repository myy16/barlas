# 🎯 BARLAS Arduino Dart Targeting System

**YOLO tabanlı dart tanıma ve Arduino Pan-Tilt lazer hedefleme sistemi**

Bu sistem, kamera ile dart'ları otomatik tespit eder ve Arduino kontrollü servo sistemle lazer hedeflemesi yapar.

## 🚀 Ana Özellikler

- **🎯 YOLO Dart Detection**: Yapay zeka ile hassas dart tanıma (0.5-0.9 güven oranı)
- **🤖 Arduino Pan-Tilt Kontrolü**: 2 servo motor ile hassas hareket
- **🔴 Otomatik Laser Hedefleme**: 2 saniye kilitlenme sonrası otomatik hedefleme
- **🎮 Hibrit Kontrol Sistemi**: Otomatik + Manuel (WASD) kontrol
- **📹 Çoklu Kamera Desteği**: USB ve dahili kamera otomatik tespiti
- **⚡ Gerçek Zamanlı İşlem**: 30 FPS kamera, 5-10 FPS YOLO
- **🔧 Plug & Play**: Arduino otomatik tespit ve bağlantı

## 📁 Dosya Yapısı

```
dart_laser_system/
├── yolo_arduino_dart_system.py    # Ana sistem
├── arduino_controller.py          # Arduino kontrol modülü  
├── arduino_pantilt.ino            # Arduino firmware
├── camera_test.py                 # Kamera test aracı
├── HARDWARE_SETUP.md              # Donanım kurulum rehberi
├── README.md                      # Bu dosya
└── requirements.txt               # Gerekli paketler
```

## 🔧 Gereksinimler

### Donanım
- **Arduino Uno** (veya uyumlu)
- **USB Kamera** veya **Dahili Kamera**
- **2x SG90 Servo Motor** (Pan-Tilt için)
- **Lazer Diod Modülü** (5V, Pin 13)
- **Jumper Kablolar** ve **Breadboard**
- **5V Güç Kaynağı** (Servo'lar için önerilir)

### Yazılım
- **Python 3.8+**
- **Arduino IDE** (Firmware için)
- **OpenCV 4.x**
- **NumPy**
- **PySerial** (Arduino iletişimi)
- **PyYAML**

## 🔌 Arduino Bağlantıları

| Bileşen | Arduino Pin | Açıklama |
|---------|-------------|----------|
| **Pan Servo** | Pin 9 | Yatay hareket servosu |
| **Tilt Servo** | Pin 10 | Dikey hareket servosu |
| **Lazer Diod** | Pin 13 | Lazer pointer (dirençle) |
| **GND** | GND | Ortak toprak |
| **VCC** | 5V | Servo beslemesi |
| **VCC** | 5V | Servo beslemesi |

## ⚙️ Kurulum

1. **Depoyu klonla:**
```bash
git clone <repo-url>
cd barlas/src/dart_laser_system
```

2. **Gerekli paketleri yükle:**
```bash
pip install opencv-python numpy pyserial PyYAML
```

3. **Arduino Firmware Yükleme:**
```bash
# Arduino IDE ile arduino_pantilt.ino dosyasını Arduino'ya yükle
# Tools → Board: Arduino Uno
# Tools → Port: COM3, COM4, COM5... (otomatik tespit edilir)
```

4. **YOLO modelini kontrol et:**
```bash
# Model yolu: ../dart_recognize/yolo_predictions.py
# Otomatik olarak import edilir
```

## 🎯 Kullanım

### 1. Kamera Testi (Önerilen)

```bash
python camera_test.py
```

**Çıktı örneği:**
```
✅ Kamera 0: 640x480 - Dahili
✅ Kamera 1: 1920x1080 - USB
```

### 2. Ana Sistem Başlatma

**USB Kamera ile:**
```bash
python yolo_arduino_dart_system.py --camera 1
```

**Dahili Kamera ile:**
```bash
python yolo_arduino_dart_system.py --camera 0
```

**Otomatik Kamera Tespiti:**
```bash
python yolo_arduino_dart_system.py
```

**Kamera Listesi:**
```bash
python yolo_arduino_dart_system.py --list-cameras
```

## 🎮 Sistem Çalışma Modları

### 🤖 Otomatik Mod (Varsayılan)
- **YOLO** sürekli dart arıyor
- **Dart bulunca** 2 saniye kilitliyor
- **Otomatik** servo hareket + lazer açılması

### 🎮 Manuel Kontrol (Hibrit)
| Tuş | Fonksiyon |
|-----|-----------|
| `w/s` | Tilt Yukarı/Aşağı |
| `a/d` | Pan Sol/Sağ |
| `space` | Lazer Aç/Kapat |
| `c` | Merkez Pozisyon |
| `q` | Çıkış |

### 🔧 Ayar Kontrolleri
| Tuş | Fonksiyon |
|-----|-----------|
| `+/-` | Güven Eşiği Ayarlama |

## 🎯 Sistem Çıktı Örnekleri

**Başarılı Başlatma:**
```
✅ YOLO Dart Recognition modülü yüklendi!
✅ Arduino Controller modülü yüklendi!
🔍 Arduino bağlantısı deneniyor: COM3
✅ Gerçek Arduino bağlandı: COM3
📹 Kamera: 1 (USB)
✅ Kamera: 1920x1080
🎯 Sistem hazır!
```

**Dart Tespiti ve Hedefleme:**
```
🎯 Dart merkezi bulundu: (245, 378), r=34
🔥 DART KİLİTLENDİ! Hough Circle Merkez: (245, 378)
🚀 Arduino'ya hedefleme komutu gönderiliyor...
🎯 HEDEF: Piksel (245, 378) -> Servo (82°, 95°)
🔄 SERVO KOMUTU: MOVE,82,95
🔴 ARDUINO KOMUTU: LASER,ON
```

## 🔧 Yapılandırma

### Hedefleme Parametreleri

```python
# yolo_arduino_dart_system.py içinde
confidence_threshold = 0.5     # Minimum dart güveni (0.0-1.0)
lock_duration = 2.0           # Kilitlenme süresi (saniye)
```

### Arduino Servo Limitleri

```cpp
// arduino_pantilt.ino içinde
const int PAN_MIN = 10;    // Pan minimum açı
const int PAN_MAX = 170;   // Pan maksimum açı  
const int TILT_MIN = 30;   // Tilt minimum açı
const int TILT_MAX = 150;  // Tilt maksimum açı
```

### Kamera Ayarları

```python
# yolo_arduino_dart_system.py içinde
self.frame_width = 640   # Kamera genişliği
self.frame_height = 480  # Kamera yüksekliği
```

### Kalibrasyon

```python
# Kamera-lazer offset ayarı
controller.calibrate_offset(pixel_offset_x=10, pixel_offset_y=-5)
```

## ❌ Sorun Giderme

### Arduino Bulunamıyor
```bash
# COM portları kontrol et
python -c "import serial.tools.list_ports; [print(p.device, p.description) for p in serial.tools.list_ports.comports()]"

# Arduino IDE'de bağlantıyı test et
# Device Manager'da Arduino Uno var mı kontrol et
```

### Kamera Açılmıyor
```bash
# Kameraları test et
python camera_test.py

# Farklı indeks dene
python yolo_arduino_dart_system.py --camera 1
### YOLO Modülü Bulunamıyor
```bash
# dart_recognize klasörünü kontrol et
ls -la ../dart_recognize/

# Python path kontrol et
python -c "import sys; print('\n'.join(sys.path))"
```

### Servo Çalışmıyor
- Servo beslemesini kontrol et (5V)
- Arduino pin bağlantıları doğru mu?
- Firmware yüklenmiş mi?

## 📊 Performans İpuçları

### FPS Artırma
```python
# Kamera buffer azalt
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# YOLO'yu her 2 frame'de bir çalıştır
if frame_count % 2 == 0:
    detect_darts(frame)
```

### CPU Optimizasyonu
- **Düşük çözünürlük** kullan (640x480)
- **YOLO skip** aktif et
- **Background thread** kullan

## 🔗 Dosya Yolları Kontrol

### YOLO Model Yolları
```python
# Otomatik import path'i
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dart_recognize.yolo_predictions import YOLOPredictions

# Başarılı import çıktısı:
# ✅ YOLO Dart Recognition modülü yüklendi!
```

### Arduino Controller Yolu
```python
# Aynı dizinde olmalı
from arduino_controller import ArduinoPanTiltController

# Başarılı import çıktısı:  
# ✅ Arduino Controller modülü yüklendi!
```

## 🚧 Gelecek Geliştirmeler

- [ ] **Multi-dart hedefleme** (çoklu lazer)
- [ ] **Hareket tahmin algoritması** (moving dart tracking)
- [ ] **Web arayüzü** (uzaktan kontrol)
- [ ] **Auto-calibration** (kamera-lazer hizalama)
- [ ] **ML-based targeting optimization**

## 📞 Destek ve Dokümantasyon

**Detaylı kurulum rehberi:** `HARDWARE_SETUP.md`  
**Sistem çalışma mantığı:** `sistem_calisma_mantigi.md`  
**Hibrit kontrol:** `hibrit_kontrol_sistemi.md`

---

🎯 **BARLAS Arduino Dart Targeting System v2.0**  
*Raspberry Pi'dan Arduino'ya geçiş tamamlandı!*
