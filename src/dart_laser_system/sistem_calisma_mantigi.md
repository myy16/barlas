# 🎯 BARLAS Arduino + Kamera Eş Güdüm Sistemi

## 🔄 Sistem Akışı

```
1. BAŞLATMA AŞAMASI:
   ├── Kamera Tespiti (0=Dahili, 1=USB, 2=...) 
   ├── Arduino Tespiti (COM3, COM4, COM5...)
   └── YOLO Model Yükleme

2. ÇALIŞMA DÖNGÜSÜ (30 FPS):
   ├── [Kamera] Frame yakala
   ├── [YOLO] Dart tespit et
   ├── [HoughCircle] Merkez bul  
   ├── [Arduino] Servo komutları gönder
   └── [Görsel] Ekrana çiz
```

## 📹 Kamera Otomatik Seçimi

### Sistem nasıl karar veriyor?

```python
# Kamera tespiti sırası:
cv2.VideoCapture(0)  # Dahili kamera (laptop)
cv2.VideoCapture(1)  # USB kamera #1
cv2.VideoCapture(2)  # USB kamera #2
# ...
```

### Manuel seçim:
```bash
# Dahili kamera kullan
python yolo_arduino_dart_system.py --camera 0

# USB kamera kullan  
python yolo_arduino_dart_system.py --camera 1

# Kamera listele
python yolo_arduino_dart_system.py --list-cameras
```

## 🤖 Arduino Otomatik Bağlantı

### Sistem nasıl Arduino buluyor?

```python
# COM port tarama sırası:
for port in ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']:
    try:
        arduino = ArduinoPanTiltController(port=port)
        if arduino.connect():
            print(f"✅ Arduino bulundu: {port}")
            return arduino
    except:
        continue

# Arduino bulunamazsa simülatör kullan
return ArduinoSimulator()
```

## ⚙️ Eş Güdüm Döngüsü

```python
def run_dart_targeting(self):
    while True:
        # 1. KAMERA - Frame yakala (33ms - 30 FPS)
        ret, frame = self.cap.read()
        
        # 2. YOLO - Dart tespit (100ms)
        dart_detections = self.yolo_detector.get_detections(frame)
        
        # 3. HOUGH CIRCLE - Merkez bul (50ms)
        if dart_detections:
            center = self.hough_detector.find_dart_center(frame, bbox)
            
            # 4. ARDUINO - Servo komutları (10ms)
            if dart_locked_for_2_seconds:
                self.arduino_controller.aim_at_pixel(center_x, center_y)
                self.arduino_controller.enable_laser()
        
        # 5. GÖRSEL - Ekrana çiz (16ms)
        cv2.imshow('BARLAS Dart Targeting', frame)
        
        # Toplam: ~200ms per frame = 5 FPS (YOLO yavaş)
```

## 🎯 Koordinat Dönüşümü

```
KAMERA FRAME    →    ARDUINO SERVO
===============     ================
Piksel (320,240) →  Pan: 90°, Tilt: 90° (Merkez)
Piksel (480,180) →  Pan: 60°, Tilt: 105° (Sağ-Yukarı)  
Piksel (160,300) →  Pan: 120°, Tilt: 75° (Sol-Aşağı)
```

## 📊 Performans Optimizasyonu

### FPS Artırma Yöntemleri:
1. **YOLO Skip**: Her 2 frame'de bir çalıştır
2. **Küçük Çözünürlük**: 416x416 YOLO input
3. **Buffer Azaltma**: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
4. **Threading**: Kamera ve Arduino ayrı thread'lerde

### Örnek İyileştirme:
```python
frame_count = 0
while True:
    ret, frame = self.cap.read()
    
    # YOLO'yu her frame'de çalıştırma
    if frame_count % 2 == 0:  # Her 2 frame'de bir
        dart_detections = self.yolo_detector.get_detections(frame)
    
    frame_count += 1
```

## 🔌 Donanım Bağlantı Kontrolleri

### Kamera Kontrolü:
```python
def check_cameras():
    available = []
    for i in range(5):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                available.append(f"Kamera {i}: {w}x{h}")
        cap.release()
    return available
```

### Arduino Kontrolü:
```python
def check_arduino_ports():
    import serial.tools.list_ports
    ports = []
    for port in serial.tools.list_ports.comports():
        if 'Arduino' in port.description:
            ports.append(port.device)
    return ports
```

## 🎮 Gerçek Zamanlı Kontrol

```
Kamera Frame Rate: 30 FPS (33ms/frame)
YOLO Processing:   5-10 FPS (100-200ms)
Arduino Commands:  100 FPS (10ms/komut)
Servo Movement:    200ms (fiziksel hareket)
Laser Response:    1ms (anında)
```

## 🚀 Başlatma Senaryoları

### Senaryo 1: Dahili Kamera + Arduino
```bash
python yolo_arduino_dart_system.py
# Çıktı:
✅ Kamera 0: 640x480 - Dahili
✅ Arduino bulundu: COM3
🎯 Sistem hazır!
```

### Senaryo 2: USB Kamera + Arduino
```bash
python yolo_arduino_dart_system.py --camera 1
# Çıktı:  
✅ Kamera 1: 1920x1080 - USB
✅ Arduino bulundu: COM4
🎯 Sistem hazır!
```

### Senaryo 3: Sadece Kamera (Arduino yok)
```bash
python yolo_arduino_dart_system.py
# Çıktı:
✅ Kamera 0: 640x480 - Dahili
⚠️ Arduino bulunamadı, simülatör kullanılıyor
🎮 Sistem hazır!
```

## 🎯 Test Komutları

```bash
# Kameraları listele
python camera_test.py

# Dahili kamera ile test
python yolo_arduino_dart_system.py --camera 0

# USB kamera ile test  
python yolo_arduino_dart_system.py --camera 1

# Arduino portları listele
python -c "import serial.tools.list_ports; [print(p.device, p.description) for p in serial.tools.list_ports.comports()]"
```
