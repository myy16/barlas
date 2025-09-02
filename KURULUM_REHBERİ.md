# 🚀 BARLAS SİSTEMİ KURULUM VE ÇALIŞTIRMA REHBERİ

Bu rehber, BARLAS Dart Laser Targeting Sistemini sıfırdan kurup çalıştırmak için hazırlanmıştır.

## 📋 ÖNGEREKLER

### 1. Donanım Gereksinimleri
- ✅ Arduino (Uno/Mega/Nano)
- ✅ USB kablosu (Arduino bağlantısı için)
- ✅ Pan-Tilt servo mekanizması
- ✅ Lazer modülü
- ✅ BTS7960 motor sürücü (isteğe bağlı)
- ✅ Servo fren sistemi (isteğe bağlı)
- ✅ Röle far sistemi (isteğe bağlı)
- ✅ Encoder sensörleri (isteğe bağlı)

### 2. Yazılım Gereksinimleri
- ✅ Python 3.8+
- ✅ Arduino IDE
- ✅ Webcam/Kamera

## 🔧 ADIM 1: ARDUINO KURULUMU

### 1.1 Arduino IDE'yi İndir ve Kur
1. https://www.arduino.cc/en/software adresinden Arduino IDE'yi indir
2. Kurulumu tamamla

### 1.2 Arduino Kodunu Yükle
1. Arduino'yu USB ile bilgisayara bağla
2. Arduino IDE'yi aç
3. `d:\barlas\src\dart_laser_system\arduino_pantilt_fixed.ino` dosyasını aç
4. **Tools > Board** sekmesinden Arduino modelini seç
5. **Tools > Port** sekmesinden Arduino portunu seç (Windows'ta COM3, COM4 vb.)
6. **Upload** butonuna bas (→ işareti)
7. "Done uploading" mesajını gör

### 1.3 Arduino Bağlantı Testi
Arduino IDE'de **Serial Monitor**'ü aç (Tools > Serial Monitor):
- Baud rate: **9600** seç
- `TEST` yaz ve Enter'a bas
- `OK` cevabını almalısın

## 🐍 ADIM 2: PYTHON KURULUMU

### 2.1 Python Kütüphanelerini Kur
Terminali aç ve şu komutları çalıştır:

```bash
# Gerekli kütüphaneleri kur
pip install opencv-python
pip install ultralytics
pip install pyserial
pip install numpy
pip install Pillow
```

### 2.2 YOLO Model Dosyasını Kontrol Et
`d:\barlas\src\dart_laser_system\best.onnx` dosyasının mevcut olduğunu kontrol et.

## 🎯 ADIM 3: SİSTEM TESTLERI

### 3.1 Basit Arduino Testi
```bash
cd d:\barlas\src\dart_laser_system
python test_system_integration.py
```

- `y` basarak fiziksel test yap
- Tüm sistem bileşenlerinin çalıştığını gör

### 3.2 Kamera Testi
```bash
python -c "import cv2; cap = cv2.VideoCapture(0); ret, frame = cap.read(); print('Kamera:', 'OK' if ret else 'HATA'); cap.release()"
```

## 🚀 ADIM 4: BARLAS SİSTEMİNİ ÇALIŞTIR

### 4.1 Ana Sistemi Başlat
```bash
cd d:\barlas\src\dart_laser_system
python yolo_arduino_dart_system.py
```

### 4.2 Sistem Ayarları
Kod başladığında:
1. **Arduino portu** otomatik tespit edilecek
2. **Kamera** otomatik açılacak
3. **YOLO modeli** yüklenecek

## 🎮 ADIM 5: KONTROLLER

### 5.1 Klavye Kontrolleri
- **`q`** - Çıkış
- **`l`** - Lazer açma/kapama (manuel)
- **`c`** - Pan-Tilt merkez pozisyonu
- **`r`** - Hedef sıfırlama
- **`w/s/a/d`** - Manuel pan-tilt hareket
- **`+/-`** - Güven eşiği ayarlama
- **`f`** - Tüm tespitleri göster/gizle
- **`x`** - Crosshair açma/kapama

### 5.2 Fare Kontrolleri
- **Sol Tık** - Manuel hedefleme

### 5.3 Arduino Komutları (Terminal)
Eğer doğrudan Arduino ile konuşmak istersen:
```bash
python -c "
import serial
ser = serial.Serial('COM3', 9600)  # Port'u değiştir
ser.write(b'TEST\n')
print(ser.readline().decode())
ser.close()
"
```

## 🔧 ADIM 6: SORUN GİDERME

### 6.1 Yaygın Hatalar ve Çözümleri

#### Arduino Bağlanamıyor
```bash
# Port'ları listele
python -c "import serial.tools.list_ports; [print(p) for p in serial.tools.list_ports.comports()]"

# Farklı port dene
# Windows: COM3, COM4, COM5...
# Linux: /dev/ttyUSB0, /dev/ttyACM0...
```

#### Kamera Açılmıyor
```bash
# Farklı kamera indeksi dene
python -c "import cv2; cap = cv2.VideoCapture(1); print(cap.isOpened()); cap.release()"
```

#### YOLO Modeli Bulunamıyor
```bash
# Model dosyası kontrolü
ls -la d:\barlas\src\dart_laser_system\best.onnx
```

### 6.2 Log Kontrolleri
Sistem çalışırken terminal çıktılarını takip et:
- **[BARLAS]** - Arduino mesajları
- **[YOLOArduinoSystem]** - Ana sistem mesajları
- **[Hedefleme]** - Targeting mesajları

## 📊 ADIM 7: PERFORMANSİ ARTIRMA

### 7.1 Optimizasyonlar
```python
# Kamera çözünürlüğü düşür (hız için)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# YOLO güven eşiği ayarla
confidence_threshold = 0.7  # Yüksek = daha az tespit, daha kesin
```

### 7.2 Donanım İpuçları
- Arduino'yu USB 3.0 portuna bağla
- Kaliteli USB kablosu kullan
- Servo'lar için ayrı güç kaynağı kullan

## 🎯 ADIM 8: İLERİ ÖZELLIKLER

### 8.1 Tam Araç Kontrolü (Varsa)
```python
# Motor kontrol
controller.move_forward(150)   # İleri git
controller.stop_motors()       # Dur
controller.brake_on()          # Freni çek
controller.headlight_on()      # Farı aç

# Sensör okuma
encoders = controller.read_encoders()
print(f"Encoder değerleri: {encoders}")
```

### 8.2 Otomatik Hedefleme
Sistem dart tespit ettiğinde otomatik olarak:
1. 🎯 Hedefe kilitlenir
2. 🔥 Lazer aktif olur
3. 📊 Hedef takip edilir

## 📞 DESTEK

### Sorun yaşarsan:
1. **Hata mesajını** tam olarak kaydet
2. **Arduino Serial Monitor** çıktısını kontrol et
3. **Python terminal** çıktısını oku
4. Bu rehberdeki adımları tekrarla

### Test komutları:
```bash
# Sistem durumu
cd d:\barlas\src\dart_laser_system
python test_system_integration.py

# Manuel Arduino test
python -c "
from arduino_controller_fixed import BarlasVehicleController
c = BarlasVehicleController(port='COM3')
if c.connect():
    print('Arduino OK')
    c.center_position()
    c.disconnect()
"
```

## ✅ BAŞARI!

Sistemi çalıştırdığında:
- 📹 Kamera görüntüsü açılacak
- 🎯 Dart tespiti çalışacak
- 🔄 Pan-Tilt sistem aktif olacak
- 🎮 Klavye/fare kontrolü hazır

**İyi shooting! 🎯**
