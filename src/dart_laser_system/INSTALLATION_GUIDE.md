# 🎯 BARLAS Arduino Dart System - Kurulum Rehberi

## 📋 Sistem Gereksinimleri

### Donanım Gereksinimleri
- **Arduino Uno R3** (veya uyumlu)
- **2x SG90 Servo Motor** (Pan-Tilt için)
- **Laser Diod Modülü** (3.3V/5V uyumlu)
- **USB Kamera** (veya laptop dahili kamerası)
- **Jumper Kablolar** ve **Breadboard**
- **5V Güç Kaynağı** (servo'lar için - opsiyonel)

### Yazılım Gereksinimleri
- **Windows 10/11** (Ana sistem)
- **Python 3.8+** (Test edildi: Python 3.12.6)
- **Arduino IDE** (firmware yüklemek için)
- **Git** (proje klonlamak için)

## 🔧 Adım Adım Kurulum

### 1. Python ve Pip Kurulumu
```bash
# Python 3.8+ yüklü olduğunu kontrol edin
python --version

# Pip güncellemesi
python -m pip install --upgrade pip
```

### 2. Gerekli Python Kütüphaneleri
```bash
# Ana kütüphaneler
pip install opencv-python>=4.8.0
pip install numpy>=1.21.0
pip install PyYAML>=6.0
pip install pyserial>=3.5

# YOLO desteği için (opsiyonel - performans artışı)
pip install onnxruntime>=1.12.0

# Geliştirme için (opsiyonel)
pip install matplotlib>=3.5.0
pip install pillow>=8.0.0
```

### 3. BARLAS Projesi Klonlama
```bash
# GitHub'dan projeyi klonlayın
git clone https://github.com/myy16/barlas.git
cd barlas
```

### 4. Arduino Firmware Yükleme

#### Arduino IDE Kurulumu:
1. [Arduino IDE](https://www.arduino.cc/en/software) indirin ve kurun
2. Arduino Uno'yu USB ile bilgisayara bağlayın
3. Arduino IDE'de **Araçlar > Board > Arduino Uno** seçin
4. **Araçlar > Port** menüsünden doğru COM portunu seçin

#### Firmware Yükleme:
1. `src/dart_laser_system/arduino_pantilt.ino` dosyasını Arduino IDE'de açın
2. **Yükle** (Upload) butonuna tıklayın
3. "Yükleme tamamlandı" mesajını bekleyin

#### Donanım Bağlantıları:
```
Arduino Pin Bağlantıları:
├── Pin 9  → Pan Servo (Turuncu kablo)
├── Pin 10 → Tilt Servo (Turuncu kablo)  
├── Pin 13 → Laser Diod (+)
├── 5V     → Servo Kırmızı kablolar
├── GND    → Servo Kahverengi kablolar + Laser (-)
└── USB    → Bilgisayar (güç + iletişim)
```

### 5. Sistem Testi

#### Kamera Testi:
```bash
cd src/dart_laser_system
python camera_test.py
```

#### Arduino Bağlantı Testi:
```bash
python -c "from arduino_controller import ArduinoPanTiltController; print('Arduino OK')"
```

#### YOLO Testi:
```bash
python -c "import sys; sys.path.append('..'); from dart_recognize.yolo_predictions import YOLOPredictions; print('YOLO OK')"
```

#### Tam Sistem Testi:
```bash
python yolo_arduino_dart_system.py --list-cameras
python yolo_arduino_dart_system.py --camera 0
```

## 🚀 Kullanım Kılavuzu

### Temel Çalıştırma:
```bash
cd src/dart_laser_system

# Dahili kamera ile çalıştır
python yolo_arduino_dart_system.py

# USB kamera ile çalıştır  
python yolo_arduino_dart_system.py --camera 1

# Mevcut kameraları listele
python yolo_arduino_dart_system.py --list-cameras
```

### Sistem Kontrolleri:
- **q**: Sistemden çık
- **space**: Manuel laser açma/kapama
- **c**: Servo'ları merkeze getir
- **w/s**: Tilt servo hareket (yukarı/aşağı)
- **a/d**: Pan servo hareket (sol/sağ)
- **+/-**: YOLO güven eşiği ayarla
- **Mouse Click**: Manuel hedefleme

### Otomatik Dart Hedefleme:
1. Sistemi başlatın
2. Kameraya dart gösterin
3. Sistem 2 saniye dart'ı takip edecek
4. Otomatik olarak laser ile hedefleyecek

## 🔍 Sorun Giderme

### Python Import Hataları:
```bash
# Kütüphane eksikse
pip install <eksik_kütüphane>

# Virtual environment kullanın (önerilen)
python -m venv barlas_env
barlas_env\Scripts\activate
pip install -r requirements.txt
```

### Arduino Bağlantı Sorunları:
1. **COM Port Kontrolü:**
   - Device Manager > Ports (COM & LPT)
   - Arduino'nun hangi COM portunda olduğunu kontrol edin

2. **Driver Problemi:**
   - CH340/FTDI driver'ları yükleyin
   - Arduino IDE'den "Burn Bootloader" deneyin

3. **Firmware Sorunu:**
   - arduino_pantilt.ino'yu tekrar yükleyin
   - Serial Monitor'da (9600 baud) mesajları kontrol edin

### Kamera Sorunları:
```bash
# Kamera kontrolü
python -c "import cv2; cap=cv2.VideoCapture(0); print('Kamera:', cap.isOpened())"

# Farklı kamera indeksleri deneyin (0,1,2...)
python yolo_arduino_dart_system.py --camera 1
```

### YOLO Model Sorunu:
1. `dart_recognize/Model/weights/best.onnx` dosyasının varlığını kontrol edin
2. `dart_recognize/data.yaml` dosyasının doğru olduğunu kontrol edin

### Performans Optimizasyonu:
```bash
# Düşük FPS sorunu için
# 1. Kamera çözünürlüğünü düşürün
# 2. YOLO confidence threshold'u artırın (+/- tuşları)
# 3. Güçlü bir GPU kullanın (NVIDIA CUDA)
```

## 📁 Proje Dosya Yapısı
```
barlas/
├── src/
│   ├── dart_recognize/
│   │   ├── Model/weights/best.onnx    # YOLO model
│   │   ├── data.yaml                  # YOLO config
│   │   └── yolo_predictions.py        # YOLO detector
│   └── dart_laser_system/
│       ├── arduino_controller.py      # Arduino interface
│       ├── arduino_pantilt.ino       # Arduino firmware
│       ├── yolo_arduino_dart_system.py # Ana sistem
│       ├── camera_test.py            # Kamera test
│       └── requirements.txt          # Python dependencies
```

## 🎯 Sistem Özellikleri
- **YOLO v5/v8 Dart Detection**: 0.5-0.9 confidence ile dart tanıma
- **Hough Circle Algorithm**: Hassas merkez tespiti
- **Arduino Pan-Tilt Control**: 2 servo ile hassas hareket
- **Laser Targeting**: Otomatik 2 saniye kilitlenme
- **Hybrid Control**: Otomatik + manuel kontrol
- **Multi-Camera Support**: USB ve dahili kamera desteği
- **Real-time Processing**: 30 FPS kamera, 5-10 FPS YOLO

## 📞 Destek
Sorun yaşarsanız:
1. Kurulum adımlarını tekrar kontrol edin
2. Python ve kütüphane versiyonlarını kontrol edin
3. Arduino bağlantı ve firmware durumunu kontrol edin
4. GitHub Issues bölümünde sorun bildirin

---
🎯 **BARLAS Arduino Dart Targeting System v1.0**  
*Successful dart detection and targeting with Arduino integration*
