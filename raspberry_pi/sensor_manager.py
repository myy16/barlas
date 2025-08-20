#!/usr/bin/env python3
"""
BARLAS Robot - Sensör Veri Toplama ve İletişim
Raspberry Pi üzerinde çalışacak ana sensör yönetim programı
"""
import time
import json
import serial
import threading
from typing import Dict, Any
import RPi.GPIO as GPIO
import numpy as np

class SensorManager:
    """Tüm sensörlerin yönetimi ve veri gönderimi"""
    
    # Ultrasonik sensör pin konfigürasyonu
    ULTRASONIC_PINS = {
        1: (17, 27),  # Ön sol
        2: (22, 23),  # Ön orta-sol
        3: (24, 25),  # Ön orta-sağ
        4: (5, 6),    # Ön sağ
        5: (13, 19),  # Arka sol
        6: (20, 21),  # Arka orta-sol
        7: (12, 16),  # Arka orta-sağ
        8: (26, 18)   # Arka sağ
    }
    
    def __init__(self, serial_port: str = '/dev/ttyACM0', baud_rate: int = 115200):
        """
        Args:
            serial_port: USB seri port
            baud_rate: Haberleşme hızı
        """
        # Seri port iletişimi
        self.serial = serial.Serial(serial_port, baud_rate)
        
        # Sensör değişkenleri
        self.ultrasonic_distances = [0.0] * 8
        self.lidar_data = None
        self.imu_data = None
        self.temperature = 0.0
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        
        # Thread kontrolü
        self.running = False
        self.read_thread = None
        self.send_thread = None
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self._setup_gpio()
        
        print("✅ Sensör yöneticisi başlatıldı")
    
    def _setup_gpio(self):
        """GPIO pinlerini ayarla"""
        for trigger_pin, echo_pin in self.ULTRASONIC_PINS.values():
            GPIO.setup(trigger_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN)
            GPIO.output(trigger_pin, False)
    
    def measure_distance(self, trigger_pin: int, echo_pin: int) -> float:
        """Ultrasonik sensörden mesafe ölç"""
        # Trigger pulse gönder
        GPIO.output(trigger_pin, True)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(trigger_pin, False)
        
        # Echo pulse'ı bekle
        start_time = time.time()
        stop_time = start_time
        timeout = start_time + 0.1  # 100ms timeout
        
        # Echo yüksek seviyeyi bekle
        while GPIO.input(echo_pin) == 0:
            start_time = time.time()
            if start_time > timeout:
                return float('inf')
        
        # Echo düşük seviyeyi bekle
        while GPIO.input(echo_pin) == 1:
            stop_time = time.time()
            if stop_time > timeout:
                return float('inf')
        
        # Mesafe hesapla (metre)
        duration = stop_time - start_time
        distance = (duration * 343.0) / 2  # Ses hızı 343 m/s
        
        return min(max(distance, 0.02), 4.0)  # 2cm - 4m arası sınırla
    
    def read_all_ultrasonics(self):
        """Tüm ultrasonik sensörleri oku"""
        for i, (trigger, echo) in self.ULTRASONIC_PINS.items():
            self.ultrasonic_distances[i-1] = self.measure_distance(trigger, echo)
    
    def read_lidar(self):
        """YDLIDAR X4'ten veri oku"""
        # TODO: YDLIDAR okuma kodunu ekle
        pass
    
    def read_imu(self):
        """MPU9250'den veri oku"""
        # TODO: IMU okuma kodunu ekle
        pass
    
    def read_temperature(self):
        """DS18B20'den sıcaklık oku"""
        # TODO: Sıcaklık okuma kodunu ekle
        pass
    
    def read_battery(self):
        """INA219'dan batarya verilerini oku"""
        # TODO: Batarya okuma kodunu ekle
        pass
    
    def _sensor_reading_loop(self):
        """Sürekli sensör okuma döngüsü"""
        while self.running:
            try:
                # Tüm sensörleri oku
                self.read_all_ultrasonics()
                self.read_lidar()
                self.read_imu()
                self.read_temperature()
                self.read_battery()
                
                time.sleep(0.01)  # 100Hz okuma
                
            except Exception as e:
                print(f"⚠️ Sensör okuma hatası: {e}")
                time.sleep(1.0)
    
    def _data_sending_loop(self):
        """Sürekli veri gönderme döngüsü"""
        while self.running:
            try:
                # Sensör verilerini JSON formatına çevir
                data = {
                    'timestamp': time.time(),
                    'ultrasonics': self.ultrasonic_distances,
                    'lidar': self.lidar_data,
                    'imu': self.imu_data,
                    'temperature': self.temperature,
                    'battery': {
                        'voltage': self.battery_voltage,
                        'current': self.battery_current
                    }
                }
                
                # JSON string oluştur ve gönder
                json_str = json.dumps(data) + '\n'
                self.serial.write(json_str.encode())
                
                time.sleep(0.02)  # 50Hz gönderim
                
            except Exception as e:
                print(f"⚠️ Veri gönderme hatası: {e}")
                time.sleep(1.0)
    
    def start(self):
        """Sensör okuma ve veri göndermeyi başlat"""
        if not self.running:
            self.running = True
            
            # Okuma thread'ini başlat
            self.read_thread = threading.Thread(target=self._sensor_reading_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            # Gönderme thread'ini başlat
            self.send_thread = threading.Thread(target=self._data_sending_loop)
            self.send_thread.daemon = True
            self.send_thread.start()
            
            print("✅ Sensör okuma ve veri gönderme başlatıldı")
    
    def stop(self):
        """Sensör okuma ve veri göndermeyi durdur"""
        self.running = False
        
        if self.read_thread:
            self.read_thread.join()
        if self.send_thread:
            self.send_thread.join()
            
        self.serial.close()
        GPIO.cleanup()
        print("⏹️ Sensör yöneticisi durduruldu")

if __name__ == '__main__':
    try:
        # Sensör yöneticisini başlat
        manager = SensorManager()
        manager.start()
        
        # Ana program döngüsü
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Program durduruluyor...")
    finally:
        if 'manager' in locals():
            manager.stop()
