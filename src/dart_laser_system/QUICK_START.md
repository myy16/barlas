# 🎯 BARLAS Arduino Dart System - Hızlı Başlangıç

## ⚡ 5 Dakikada Kurulum

### 1. Gereksinimler Kontrolü
```bash
# Python 3.8+ kontrol et
python --version

# Git ile projeyi klon et
git clone https://github.com/myy16/barlas.git
cd barlas/src/dart_laser_system
```

### 2. Otomatik Kurulum (Windows)
```powershell
# PowerShell'i yönetici olarak çalıştır
PowerShell -ExecutionPolicy Bypass -File install_windows.ps1
```

### 3. Otomatik Kurulum (Linux/Mac)
```bash
# Script'i çalıştırılabilir yap
chmod +x install_linux.sh

# Kurulumu başlat
./install_linux.sh
```

### 4. Manuel Kurulum
```bash
# Kütüphaneleri yükle
pip install -r requirements.txt

# Import testleri
python -c "import cv2, numpy, serial; print('OK')"
```

### 5. Arduino Hazırlığı
1. **Arduino IDE** indir ve kur
2. **arduino_pantilt.ino** dosyasını aç
3. Arduino Uno'ya yükle
4. Serial Monitor'da "BARLAS Arduino Pan-Tilt Ready" mesajını gör

### 6. Donanım Bağlantıları
```
Arduino Pin Bağlantıları:
├── Pin 9  → Pan Servo
├── Pin 10 → Tilt Servo  
├── Pin 13 → Laser Diod
├── 5V     → Servo +
└── GND    → Servo - & Laser -
```

### 7. İlk Test
```bash
# Kameraları listele
python yolo_arduino_dart_system.py --list-cameras

# Sistemi başlat
python yolo_arduino_dart_system.py
```

## 🎮 Kontroller
- **q**: Çıkış
- **space**: Laser aç/kapat
- **c**: Merkez pozisyon
- **wasd**: Manuel hareket
- **+/-**: YOLO threshold

## 🔧 Sorun Giderme

### Python Import Hatası
```bash
pip install opencv-python numpy pyserial PyYAML
```

### Arduino Bulunamıyor
1. USB kablosunu kontrol et
2. Arduino IDE'den port seç
3. Firmware'i tekrar yükle

### Kamera Çalışmıyor
```bash
# Farklı kamera dene
python yolo_arduino_dart_system.py --camera 1
```

### YOLO Model Eksik
1. `dart_recognize/Model/weights/best.onnx` dosyasını kontrol et
2. Git repository'nin tamamını clone ettiğinden emin ol

## 📊 Sistem Gereksinimleri
- **CPU**: Intel i5 veya AMD Ryzen 5 (minimum)
- **RAM**: 4GB (8GB önerilen)
- **Python**: 3.8+ 
- **Arduino**: Uno R3 veya uyumlu
- **Kamera**: USB veya dahili webcam

## 🎯 Başarı Kriterleri
Sistem çalışıyorsa şunları göreceksiniz:
1. ✅ Kamera görüntüsü açılır
2. ✅ YOLO dart tespiti çalışır  
3. ✅ Arduino servo'lar hareket eder
4. ✅ Laser otomatik hedefleme yapar

---
**BARLAS Team** - Arduino tabanlı dart hedefleme sistemi
