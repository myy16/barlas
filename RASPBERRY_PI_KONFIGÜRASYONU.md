# 🍓 BARLAS RASPBERRY PI KONFIGÜRASYONU

## 📡 RASPBERRY PI PIN ATLASI

Raspberry Pi üzerindeki sensörler ve bağlantılar:

### 🌡️ BME SENSÖRÜ (I2C)
```python
BME280/BME680 → I2C Bağlantısı
SDA → GPIO 2  (Pin 3)
SCL → GPIO 3  (Pin 5)
VCC → 3.3V   (Pin 1)
GND → Ground (Pin 6)
```

### 📏 ULTRASONİK SENSÖRLER (GPIO)
Toplam 6 adet ultrasonik sensör:

#### Sensör 1 (Ön-Sol)
```python
Trig → GPIO 8  (Pin 24)
Echo → GPIO 10 (Pin 19)
```

#### Sensör 2 (Ön-Orta)  
```python
Trig → GPIO 11 (Pin 23)
Echo → GPIO 13 (Pin 33)
```

#### Sensör 3 (Ön-Sağ)
```python
Trig → GPIO 16 (Pin 36)
Echo → GPIO 18 (Pin 12)
```

#### Sensör 4 (Arka-Sol)
```python
Trig → GPIO 29 (Pin 40)
Echo → GPIO 31 (Pin 28)
```

#### Sensör 5 (Arka-Orta)
```python
Trig → GPIO 32 (Pin 32)
Echo → GPIO 33 (Pin 35)
```

#### Sensör 6 (Arka-Sağ)
```python
Trig → GPIO 35 (Pin 37)
Echo → GPIO 37 (Pin 26)
```

## 🐍 PYTHON KOD ÖRNEKLER

### BME Sensör Okuma:
```python
import board
import adafruit_bme280

# I2C bağlantısı
i2c = board.I2C()  # SDA=GPIO2, SCL=GPIO3
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# Veri okuma
temperature = bme280.temperature
humidity = bme280.humidity
pressure = bme280.pressure

print(f"Sıcaklık: {temperature:.1f}°C")
print(f"Nem: {humidity:.1f}%")
print(f"Basınç: {pressure:.1f} hPa")
```

### Ultrasonik Sensör Okuma:
```python
import RPi.GPIO as GPIO
import time

# Pin konfigürasyonu
TRIG_PINS = [8, 11, 16, 29, 32, 35]
ECHO_PINS = [10, 13, 18, 31, 33, 37]

def setup_ultrasonic():
    GPIO.setmode(GPIO.BCM)
    
    for trig in TRIG_PINS:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    
    for echo in ECHO_PINS:
        GPIO.setup(echo, GPIO.IN)
    
    time.sleep(2)

def read_distance(trig_pin, echo_pin):
    # Trigger pulse
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    
    # Echo measurement
    start_time = time.time()
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
    
    stop_time = time.time()
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()
    
    # Distance calculation
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2
    
    return distance

# Kullanım
setup_ultrasonic()

for i in range(6):
    distance = read_distance(TRIG_PINS[i], ECHO_PINS[i])
    print(f"Sensör {i+1}: {distance:.1f} cm")
    time.sleep(0.1)
```

## 🔌 ARDUINO-RASPBERRY PI İLETİŞİMİ

### Arduino'dan Raspberry Pi'ye Veri:
```python
import serial
import json

# Arduino ile serial iletişim
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def get_arduino_data():
    # Arduino'dan encoder verisi al
    arduino.write(b'GET_ENCODERS\n')
    response = arduino.readline().decode().strip()
    
    if "ENCODER1:" in response:
        # Parse encoder data
        parts = response.split(",")
        encoder1 = int(parts[0].split(":")[1])
        encoder2 = int(parts[1].split(":")[1])
        
        return {"encoder1": encoder1, "encoder2": encoder2}
    
    return None

# Raspberry Pi sensör verisi ile Arduino verisi birleştir
def get_all_sensor_data():
    data = {}
    
    # BME verisi
    data["temperature"] = bme280.temperature
    data["humidity"] = bme280.humidity
    data["pressure"] = bme280.pressure
    
    # Ultrasonik veriler
    data["distances"] = []
    for i in range(6):
        distance = read_distance(TRIG_PINS[i], ECHO_PINS[i])
        data["distances"].append(distance)
    
    # Arduino encoder verisi
    arduino_data = get_arduino_data()
    if arduino_data:
        data.update(arduino_data)
    
    return data

# JSON formatında tüm veri
sensor_data = get_all_sensor_data()
print(json.dumps(sensor_data, indent=2))
```

## 🌐 WEB API SERVİSİ

### Flask ile sensor API:
```python
from flask import Flask, jsonify
import threading
import time

app = Flask(__name__)

# Global sensor data
current_sensor_data = {}

def sensor_thread():
    """Arka planda sürekli sensör okuma"""
    global current_sensor_data
    
    while True:
        try:
            current_sensor_data = get_all_sensor_data()
            time.sleep(0.1)  # 10Hz güncelleme
        except Exception as e:
            print(f"Sensor error: {e}")

# API endpoints
@app.route('/sensors')
def get_sensors():
    return jsonify(current_sensor_data)

@app.route('/sensors/bme')
def get_bme():
    return jsonify({
        "temperature": current_sensor_data.get("temperature"),
        "humidity": current_sensor_data.get("humidity"),
        "pressure": current_sensor_data.get("pressure")
    })

@app.route('/sensors/ultrasonic')
def get_ultrasonic():
    return jsonify({
        "distances": current_sensor_data.get("distances", [])
    })

@app.route('/sensors/encoders')
def get_encoders():
    return jsonify({
        "encoder1": current_sensor_data.get("encoder1"),
        "encoder2": current_sensor_data.get("encoder2")
    })

if __name__ == '__main__':
    # Sensor thread başlat
    sensor_thread_obj = threading.Thread(target=sensor_thread)
    sensor_thread_obj.daemon = True
    sensor_thread_obj.start()
    
    # Web server başlat
    app.run(host='0.0.0.0', port=5000)
```

## 📊 ENTEGRASYONLU SİSTEM

### Arduino + Raspberry Pi Tam Entegrasyonu:
```python
class BarlasFullSystem:
    def __init__(self):
        # Arduino bağlantısı
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        
        # Raspberry Pi sensörleri
        self.setup_raspberry_sensors()
        
    def setup_raspberry_sensors(self):
        # BME280 setup
        i2c = board.I2C()
        self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
        
        # Ultrasonik setup
        setup_ultrasonic()
    
    def get_full_system_status(self):
        # Arduino durumu
        self.arduino.write(b'STATUS\n')
        arduino_status = self.arduino.readline().decode()
        
        # Raspberry Pi sensör verisi
        rasp_data = get_all_sensor_data()
        
        return {
            "arduino": arduino_status,
            "raspberry_pi": rasp_data,
            "timestamp": time.time()
        }
    
    def autonomous_navigation(self):
        """Otonom navigasyon - sensör füzyonu"""
        while True:
            # Tüm sensör verilerini al
            data = self.get_full_system_status()
            
            # Engel tespiti (ultrasonik)
            distances = data["raspberry_pi"]["distances"]
            min_distance = min(distances)
            
            if min_distance < 20:  # 20cm'den yakın engel
                # Dur
                self.arduino.write(b'MOTOR_STOP\n')
                print("Engel tespit edildi, duruldu!")
                
            elif min_distance > 50:  # Güvenli mesafe
                # İleri git
                self.arduino.write(b'MOTOR_FORWARD,100\n')
                print("Güvenli, ileri gidiliyor")
            
            time.sleep(0.1)

# Sistem başlat
system = BarlasFullSystem()
```

Bu konfigürasyon dosyaları ile **tam entegrasyonlu** BARLAS sistemi hazır! 🚗🎯🍓
