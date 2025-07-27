# 🔧 BARLAS Arduino Dart System - Sorun Giderme Kılavuzu

## 🐍 Python ve Kütüphane Sorunları

### "ModuleNotFoundError: No module named 'cv2'"
```bash
# Çözüm:
pip install opencv-python
# veya
pip install opencv-python-headless
```

### "ModuleNotFoundError: No module named 'serial'"
```bash
# Çözüm:
pip install pyserial
```

### "ImportError: numpy" 
```bash
# Çözüm:
pip install numpy>=1.21.0
```

### Python Versiyonu Eski
```bash
# Python 3.8+ gerekli
python --version

# Güncelleme (Windows):
# Python.org'dan son sürümü indirin

# Güncelleme (Ubuntu):
sudo apt update
sudo apt install python3.10
```

## 🤖 Arduino Bağlantı Sorunları

### Arduino Bulunamıyor
**Belirtiler:**
- "Arduino bulunamadı, simülatör kullanılıyor" mesajı
- COM port listesi boş

**Çözümler:**
1. **USB Kablosu Kontrolü:**
   - Veri kablosu kullanın (şarj kablosu değil)
   - Kabloyu değiştirin
   - Farklı USB portuna takın

2. **Driver Kontrolü:**
   ```bash
   # Windows Device Manager'da kontrol edin
   # "Ports (COM & LPT)" bölümünde Arduino görünmeli
   ```

3. **Arduino IDE Testi:**
   - Arduino IDE'yi açın
   - Tools > Port menüsünden Arduino'yu seçin
   - File > Examples > Basics > Blink yükleyin
   - Çalışırsa driver sorunu yok

### "Permission denied" Hatası (Linux)
```bash
# Kullanıcıyı dialout grubuna ekleyin
sudo usermod -a -G dialout $USER

# Çıkış yapıp tekrar giriş yapın
# veya
sudo chmod 666 /dev/ttyUSB0  # veya ttyACM0
```

### Arduino Firmware Yükleme Sorunu
1. **Arduino IDE'de:**
   - Tools > Board: "Arduino Uno"
   - Tools > Port: Doğru COM port
   - Sketch > Upload

2. **Hata alırsanız:**
   - Arduino'yu USB'den çıkarıp takın
   - Farklı USB kablosu deneyin
   - Arduino IDE'yi yeniden başlatın

## 📹 Kamera Sorunları

### "Kamera açılamadı" Hatası
**Çözümler:**
1. **Farklı kamera indeksi deneyin:**
   ```bash
   python yolo_arduino_dart_system.py --camera 0  # Dahili
   python yolo_arduino_dart_system.py --camera 1  # USB
   python yolo_arduino_dart_system.py --camera 2  # Diğer
   ```

2. **Kamera listesini kontrol edin:**
   ```bash
   python yolo_arduino_dart_system.py --list-cameras
   ```

3. **Başka uygulama kamerayı kullanıyor olabilir:**
   - Skype, Teams, Zoom gibi uygulamaları kapatın
   - Windows Camera uygulamasını kapatın

### Kamera Görüntüsü Gelmez
```bash
# Kamera testi
python camera_test.py

# Manuel test
python -c "import cv2; cap = cv2.VideoCapture(0); print(cap.read())"
```

### Düşük FPS / Yavaş Görüntü
1. **Kamera çözünürlüğünü düşürün** (kodda 640x480)
2. **YOLO confidence threshold artırın** (+/- tuşları)
3. **Güçlü bilgisayar kullanın**

## 🎯 YOLO Model Sorunları

### "YOLO modülü bulunamadı" Hatası
**Kontrol listesi:**
1. `dart_recognize` klasörü var mı?
2. `dart_recognize/yolo_predictions.py` dosyası var mı?
3. `dart_recognize/Model/weights/best.onnx` modeli var mı?

**Çözüm:**
```bash
# Tam repository'yi clone edin
git clone https://github.com/myy16/barlas.git

# Doğru dizinde olduğunuzdan emin olun
cd barlas/src/dart_laser_system
```

### YOLO Modeli Yavaş
1. **ONNX Runtime yükleyin:**
   ```bash
   pip install onnxruntime
   ```

2. **GPU desteği (NVIDIA kartlar için):**
   ```bash
   pip install onnxruntime-gpu
   ```

## 🔗 Servo Motor Sorunları

### Servo'lar Hareket Etmiyor
1. **Güç Kontrolü:**
   - 5V bağlantısını kontrol edin
   - Ayrı güç kaynağı kullanın (önerilen)
   - GND bağlantılarını kontrol edin

2. **Bağlantı Kontrolü:**
   ```
   Pan Servo  → Arduino Pin 9
   Tilt Servo → Arduino Pin 10
   Laser      → Arduino Pin 13
   ```

3. **Arduino Serial Monitor:**
   - 9600 baud rate
   - "MOVE,90,90" komutu gönderin
   - "OK - Moved" cevabını bekleyin

### Servo'lar Titreşiyor
1. **Güç kaynağı yetersiz** - Ayrı 5V adaptör kullanın
2. **Bağlantı gevşek** - Kabloları kontrol edin
3. **PWM girişimi** - Başka PWM cihazları uzaklaştırın

## 🔴 Laser Sorunları

### Laser Çalışmıyor
1. **Pin 13 bağlantısını kontrol edin**
2. **Laser polaritesini kontrol edin** (+/- doğru bağlı mı?)
3. **Arduino Serial Monitor'da test edin:**
   ```
   LASER,ON   → "OK - Laser ON"
   LASER,OFF  → "OK - Laser OFF"
   ```

### Laser Sürekli Yanıyor
1. **Kodu kontrol edin** - otomatik kapanma var mı?
2. **Manual kontrol:** Space tuşu ile aç/kapat
3. **Acil durum:** Arduino'yu USB'den çıkarın

## 💻 Sistem Performans Sorunları

### Yavaş İşlem / Düşük FPS
1. **CPU Kullanımı:**
   - Task Manager'da Python CPU kullanımını kontrol edin
   - %100'e yakınsa normal (YOLO yoğun işlem)

2. **Bellek Sorunu:**
   - 4GB+ RAM önerilir
   - Gereksiz programları kapatın

3. **Optimizasyon:**
   ```bash
   # YOLO confidence threshold artırın
   # Sistem çalışırken +/- tuşları ile ayarlayın
   ```

### Sistem Donması
1. **Ctrl+C** ile çıkış yapın
2. **Task Manager'dan Python'u sonlandırın**
3. **Arduino'yu USB'den çıkarıp takın**

## 🔍 Debug ve Test Komutları

### Sistem Durumu Kontrolü
```bash
# Python kütüphaneleri
python -c "import cv2, numpy, serial; print('Imports OK')"

# Arduino bağlantısı
python -c "from arduino_controller import ArduinoPanTiltController; print('Arduino OK')"

# YOLO modeli
python -c "import sys; sys.path.append('..'); from dart_recognize.yolo_predictions import YOLOPredictions; print('YOLO OK')"
```

### Detaylı Log
```bash
# Verbose modda çalıştırın
python yolo_arduino_dart_system.py --camera 0 -v
```

### Arduino Serial Debug
```bash
# Arduino IDE Serial Monitor
# 9600 baud rate
# Test komutları:
TEST          → "OK - Arduino Ready"
STATUS        → "OK - Pan:90,Tilt:90,Laser:OFF"
MOVE,45,60    → "OK - Moved"
CENTER        → "OK - Centered"
```

## 📞 Yardım Alma

### Log Dosyaları
```bash
# Hata mesajlarını kaydedin
python yolo_arduino_dart_system.py 2>&1 | tee error.log
```

### Sistem Bilgileri
```bash
# Python versiyonu
python --version

# Kütüphane versiyonları  
pip list | grep -E "(opencv|numpy|serial)"

# İşletim sistemi
# Windows: ver
# Linux: lsb_release -a
```

### Destek Kanalları
1. **GitHub Issues:** Detaylı hata raporu açın
2. **README.md:** Temel kullanım talimatları
3. **INSTALLATION_GUIDE.md:** Kurulum detayları

---
🎯 **BARLAS Support Team**  
*Her sorunun bir çözümü vardır!*
