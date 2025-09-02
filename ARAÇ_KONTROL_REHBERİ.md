# 🚗 BARLAS ARAÇ KONTROLÜ - ARDUINO KOMUT REHBERİ

Bu rehber, BARLAS sistemini gerçek araç üzerinde çalıştırmak için Arduino komutlarını açıklar.

## 🎯 ARAÇ SİSTEM BİLEŞENLERİ

### 🔧 **Donanım Bağlantıları:**
```
✅ Pan-Tilt Servolar    → Pin 9, 10
✅ BTS7960 Motor Sürücü → Pin 2,3,4,5 (R_EN, L_EN, RPWM, LPWM)
✅ Servo Fren Sistemi   → Pin 6
✅ Röle Far Sistemi     → Pin 7
✅ Encoder Sensörleri   → Pin 18,19,20,21 (Interrupt)
✅ Lazer Modülü         → Pin 13
```

## 🎮 MANUEL ARAÇ KOMUTLARI

### 1. PAN-TILT KONTROL
```python
from arduino_controller_fixed import BarlasVehicleController

# Bağlantı kur
controller = BarlasVehicleController(port="COM3", baud_rate=9600)
controller.connect()

# Pan-Tilt hareket
controller.move_to_position(90, 90)    # Merkez pozisyon
controller.move_to_position(45, 120)   # Sol-Yukarı
controller.center_position()           # Merkez dön
```

### 2. HAREKET KONTROLÜ
```python
# İleri hareket (0-255 hız)
controller.move_forward(100)    # Yavaş ileri
controller.move_forward(200)    # Hızlı ileri

# Geri hareket
controller.move_backward(100)   # Yavaş geri
controller.move_backward(150)   # Orta hızda geri

# Dur
controller.stop_motors()        # Motorları durdur
```

### 3. FREN SİSTEMİ
```python
# Fren çekme
controller.brake_on()          # Freni çek
controller.brake_off()         # Freni bırak

# Acil fren (motor+fren+far)
controller.emergency_brake()   # TÜM GÜVENLİK SİSTEMLERİ
```

### 4. FAR SİSTEMİ
```python
# Far kontrolü
controller.headlight_on()     # Farı aç
controller.headlight_off()    # Farı kapat
```

### 5. ENCODER OKUMA
```python
# Encoder değerleri oku
encoders = controller.read_encoders()
print(f"Sol Encoder: {encoders['encoder1']}")
print(f"Sağ Encoder: {encoders['encoder2']}")

# Encoder sıfırla
controller.reset_encoders()
```

## 🚀 GERÇEK ARAÇ ÜZERİNDE ÇALIŞTIRMA

### ADIM 1: Arduino Hazırlığı
```bash
# 1. Arduino'yu araca monte et
# 2. Tüm sensör bağlantılarını yap
# 3. Güç kaynaklarını bağla
# 4. USB kablosu ile bilgisayara bağla
```

### ADIM 2: Kod Yükleme
```bash
# Arduino IDE'de arduino_pantilt_fixed.ino'yu aç
# Port seç (Tools → Port → COM3)
# Upload et (Ctrl+U)
# Serial Monitor'de "BARLAS Arduino Ready - Full System" mesajını bekle
```

### ADIM 3: Sistem Testi
```bash
cd d:\barlas\src\dart_laser_system
python araç_test_modu.py  # Aşağıda oluşturacağım
```

## 🛠️ MANUEL TEST KOMUTLARI

### Serial Monitor'de Doğrudan Test:
```
TEST                    → "OK" döner
STATUS                  → Sistem durumunu gösterir
MOVE,90,90             → Pan-Tilt merkez
LASER,ON               → Lazer aç
LASER,OFF              → Lazer kapat
MOTOR_FORWARD,100      → İleri git (hız 100)
MOTOR_STOP             → Dur
BRAKE_ON               → Fren çek
BRAKE_OFF              → Fren bırak
HEADLIGHT_ON           → Far aç
HEADLIGHT_OFF          → Far kapat
GET_ENCODERS           → Encoder değerleri
EMERGENCY_BRAKE        → ACİL FREN!
```

## ⚠️ GÜVENLİK PROTOKOLÜ

### BAŞLATMA ÖNCESİ KONTROLLER:
```
✅ Araç güvenli alanda (kapalı alan)
✅ Fren sistemi test edildi
✅ Acil durdurma butonu hazır
✅ Far sistemi çalışıyor
✅ Encoder değerleri okunuyor
✅ Motor sürücü sıcaklık normale
```

### ACİL DURUM KOMUTLARI:
```python
# Python'da acil durdurma
controller.emergency_brake()    # Her şeyi durdur

# Arduino Serial'de acil durdurma  
EMERGENCY_BRAKE                 # Direkt komut
```

## 🔄 OTOMATİK HEDEFLEME MODU

### YOLO + Araç Entegrasyonu:
```bash
# Ana sistemi araçta çalıştır
cd d:\barlas\src\dart_laser_system
python yolo_arduino_dart_system.py

# Otomatik moda geç
# Sistem dart tespit ettiğinde:
# 1. 🎯 Hedef takip eder
# 2. 🚗 Araca hareket komutu verir  
# 3. 🔥 Lazeri aktif eder
```

## 📊 PERFORMANS İZLEME

### Gerçek Zamanlı İzleme:
```python
import time
from arduino_controller_fixed import BarlasVehicleController

controller = BarlasVehicleController(port="COM3")
controller.connect()

# Sürekli izleme döngüsü
while True:
    # Encoder oku
    encoders = controller.read_encoders()
    
    # Sistem durumu
    status = controller.get_system_status()
    
    print(f"Encoder L/R: {encoders}")
    print(f"Durum: {status}")
    
    time.sleep(1)
```

## 🎮 JOYSTICK/GAMEPAD KONTROLÜ

### Gamepad ile Araç Kontrolü:
```python
import pygame
from arduino_controller_fixed import BarlasVehicleController

# Gamepad başlat
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    controller = BarlasVehicleController(port="COM3")
    controller.connect()
    
    while True:
        pygame.event.pump()
        
        # Sol analog çubuk - hareket
        y_axis = joystick.get_axis(1)  # İleri/geri
        
        if y_axis < -0.5:  # İleri
            speed = int(abs(y_axis) * 255)
            controller.move_forward(speed)
        elif y_axis > 0.5:  # Geri  
            speed = int(y_axis * 255)
            controller.move_backward(speed)
        else:  # Dur
            controller.stop_motors()
        
        # Butonlar
        if joystick.get_button(0):  # A butonu - fren
            controller.brake_on()
        else:
            controller.brake_off()
            
        if joystick.get_button(1):  # B butonu - far
            controller.headlight_on()
        else:
            controller.headlight_off()
```

## 🏁 YARIŞMA MODU

### Otonom Yarışma Için:
```python
# Kontrol modu değiştir
controller.set_control_mode("PIXHAWK")  # Otonom mod

# Yarışma senaryosu
def yarisma_modu():
    controller.connect()
    
    # Başlangıç kontrolü
    controller.headlight_on()       # Görünürlük
    controller.brake_off()          # Freni bırak
    controller.reset_encoders()     # Sayaçları sıfırla
    
    # YOLO sistemini başlat
    # Dart tespit ettiğinde otomatik hareket
    
    # Güvenlik: 30 saniye sonra otomatik dur
    import threading
    timer = threading.Timer(30.0, controller.emergency_brake)
    timer.start()
```

## 📱 UZAKTAN KONTROL

### WiFi/Bluetooth Modülü ile:
```python
# ESP32 bridge ile uzaktan kontrol
import requests

def uzaktan_komut(komut):
    # ESP32'ye HTTP request gönder
    url = f"http://192.168.1.100/komut/{komut}"
    response = requests.get(url)
    return response.text

# Kullanım
uzaktan_komut("MOTOR_FORWARD,150")
uzaktan_komut("BRAKE_ON") 
uzaktan_komut("EMERGENCY_BRAKE")
```

## ⚡ PERFORMANS OPTİMİZASYONU

### Hız Ayarları:
```python
# Farklı hız profilleri
YAVAS_HIZ = 80      # Hassas hareket için
NORMAL_HIZ = 150    # Normal kullanım
HIZLI_HIZ = 220     # Acil durumlar
MAX_HIZ = 255       # Maksimum performans

# Kullanım
controller.move_forward(NORMAL_HIZ)
```

### Servo Ayarları:
```python
# Pan-Tilt hız optimizasyonu
controller.set_servo_speed(50)  # Yavaş hareket
controller.set_servo_speed(100) # Hızlı hareket
```

Bu rehberi takip ederek arkadaşınız BARLAS sistemini gerçek araç üzerinde güvenli şekilde çalıştırabilir! 🚗🎯
