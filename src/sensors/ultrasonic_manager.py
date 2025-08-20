"""
JSN-SR04T Ultrasonik Sensör Yönetim Modülü
8 adet ultrasonik sensörün kontrolü ve engel tespiti
"""
import time
import threading
import RPi.GPIO as GPIO
import numpy as np
from typing import List, Dict, Tuple, Optional

class UltrasonicSensor:
    """Tekli JSN-SR04T ultrasonik sensör sınıfı"""
    
    def __init__(self, trigger_pin: int, echo_pin: int, sensor_id: int):
        """
        Args:
            trigger_pin (int): Trigger pin numarası
            echo_pin (int): Echo pin numarası
            sensor_id (int): Sensör ID (1-8 arası)
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.sensor_id = sensor_id
        self.last_distance = float('inf')
        self.is_obstacle = False
        self.min_distance = 20  # cm
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(trigger_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        
    def measure_distance(self) -> float:
        """Mesafe ölçümü yapar
        
        Returns:
            float: Ölçülen mesafe (cm)
        """
        # Trigger pulse gönder
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.trigger_pin, False)
        
        # Echo pulse'ı bekle
        pulse_start = time.time()
        timeout = pulse_start + 0.1  # 100ms timeout
        
        # Echo yüksek seviyeyi bekle
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return float('inf')
        
        # Echo düşük seviyeyi bekle
        pulse_end = time.time()
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return float('inf')
        
        # Mesafe hesapla
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Ses hızı / 2
        
        # Sonucu güncelle ve döndür
        self.last_distance = round(distance, 2)
        self.is_obstacle = self.last_distance < self.min_distance
        return self.last_distance
    
    def cleanup(self):
        """GPIO pinlerini temizle"""
        GPIO.cleanup([self.trigger_pin, self.echo_pin])

class UltrasonicManager:
    """8 adet JSN-SR04T ultrasonik sensör yönetimi"""
    
    # Sensör pin konfigürasyonu - (trigger_pin, echo_pin)
    SENSOR_PINS = {
        1: (17, 27),  # Ön sol
        2: (22, 23),  # Ön orta-sol
        3: (24, 25),  # Ön orta-sağ
        4: (5, 6),    # Ön sağ
        5: (13, 19),  # Arka sol
        6: (20, 21),  # Arka orta-sol
        7: (12, 16),  # Arka orta-sağ
        8: (26, 18)   # Arka sağ
    }
    
    def __init__(self):
        """UltrasonicManager başlatma"""
        self.sensors: Dict[int, UltrasonicSensor] = {}
        self.running = False
        self.measurement_thread = None
        self.lock = threading.Lock()
        self._initialize_sensors()
    
    def _initialize_sensors(self):
        """Tüm sensörleri başlat"""
        try:
            for sensor_id, (trigger, echo) in self.SENSOR_PINS.items():
                self.sensors[sensor_id] = UltrasonicSensor(trigger, echo, sensor_id)
            print(f"✅ {len(self.sensors)} ultrasonik sensör başlatıldı")
        except Exception as e:
            print(f"❌ Sensör başlatma hatası: {e}")
    
    def start_measurements(self):
        """Sürekli ölçüm döngüsünü başlat"""
        if not self.running:
            self.running = True
            self.measurement_thread = threading.Thread(target=self._measurement_loop)
            self.measurement_thread.daemon = True
            self.measurement_thread.start()
            print("✅ Ultrasonik ölçüm döngüsü başlatıldı")
    
    def stop_measurements(self):
        """Ölçüm döngüsünü durdur"""
        self.running = False
        if self.measurement_thread:
            self.measurement_thread.join()
        print("⏹️ Ultrasonik ölçüm döngüsü durduruldu")
    
    def _measurement_loop(self):
        """Sürekli ölçüm yapan döngü"""
        while self.running:
            with self.lock:
                for sensor in self.sensors.values():
                    sensor.measure_distance()
            time.sleep(0.1)  # 10Hz ölçüm
    
    def get_all_distances(self) -> Dict[int, float]:
        """Tüm sensörlerin mesafe değerlerini döndür
        
        Returns:
            Dict[int, float]: {sensor_id: mesafe} formatında sözlük
        """
        with self.lock:
            return {sid: sensor.last_distance 
                   for sid, sensor in self.sensors.items()}
    
    def get_obstacle_status(self) -> Dict[int, bool]:
        """Tüm sensörlerin engel durumlarını döndür
        
        Returns:
            Dict[int, bool]: {sensor_id: engel_var_mı} formatında sözlük
        """
        with self.lock:
            return {sid: sensor.is_obstacle 
                   for sid, sensor in self.sensors.items()}
    
    def get_front_distances(self) -> List[float]:
        """Ön sensörlerin (1-4) mesafelerini döndür"""
        distances = []
        with self.lock:
            for i in range(1, 5):
                distances.append(self.sensors[i].last_distance)
        return distances
    
    def get_rear_distances(self) -> List[float]:
        """Arka sensörlerin (5-8) mesafelerini döndür"""
        distances = []
        with self.lock:
            for i in range(5, 9):
                distances.append(self.sensors[i].last_distance)
        return distances
    
    def cleanup(self):
        """Tüm sensörleri temizle"""
        self.stop_measurements()
        for sensor in self.sensors.values():
            sensor.cleanup()
        print("🧹 Ultrasonik sensörler temizlendi")
