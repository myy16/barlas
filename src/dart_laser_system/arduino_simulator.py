"""
BARLAS Arduino Simulator
Arduino olmadan da sistem testini sağlar
"""
import time
import threading
from typing import Optional, Tuple
import random
import math

class ArduinoSimulator:
    """Arduino Pan-Tilt kontrolcüsü simulatörü"""
    
    def __init__(self, port="SIM_PORT"):
        """Arduino Simulator"""
        self.port = port
        self.is_connected = True
        
        # Servo pozisyonları
        self.current_pan = 90
        self.current_tilt = 90
        
        # Servo limitleri
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 20
        self.tilt_max = 160
        
        # Lazer durumu
        self.laser_active = False
        
        # Hareket simulasyonu
        self.movement_speed = 2.0  # derece/saniye
        self.noise_level = 0.5     # pozisyon gürültüsü
        
        print(f"[ArduinoSimulator] 🎮 Simulator başlatıldı")
        print(f"[ArduinoSimulator] Pan: {self.current_pan}°, Tilt: {self.current_tilt}°")
    
    def connect(self) -> bool:
        """Simülasyon - her zaman başarılı"""
        print("[ArduinoSimulator] ✅ Simülasyon bağlantısı sağlandı")
        time.sleep(0.5)  # Gerçek bağlantıyı taklit et
        return True
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        self.is_connected = False
        print("[ArduinoSimulator] 🔌 Simülasyon bağlantısı kapatıldı")
    
    def move_to_position(self, pan: float, tilt: float, smooth: bool = True) -> bool:
        """Belirtilen pozisyona hareket et"""
        if not self.is_connected:
            return False
        
        # Limitleri kontrol et
        pan = max(self.pan_min, min(self.pan_max, pan))
        tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        
        old_pan, old_tilt = self.current_pan, self.current_tilt
        
        if smooth:
            # Yumuşak hareket simulasyonu
            pan_diff = pan - self.current_pan
            tilt_diff = tilt - self.current_tilt
            
            # Hareket süresi hesapla
            max_movement = max(abs(pan_diff), abs(tilt_diff))
            movement_time = max_movement / self.movement_speed
            
            # Aşamalı hareket
            steps = max(1, int(movement_time * 10))  # 100ms adımlar
            
            for step in range(steps + 1):
                progress = step / steps
                
                # Yumuşak interpolasyon
                current_pan = old_pan + pan_diff * progress
                current_tilt = old_tilt + tilt_diff * progress
                
                # Gürültü ekle
                noise_pan = random.uniform(-self.noise_level, self.noise_level)
                noise_tilt = random.uniform(-self.noise_level, self.noise_level)
                
                self.current_pan = current_pan + noise_pan
                self.current_tilt = current_tilt + noise_tilt
                
                time.sleep(0.1)  # Hareket hızı
            
        else:
            # Anında hareket
            self.current_pan = pan + random.uniform(-self.noise_level, self.noise_level)
            self.current_tilt = tilt + random.uniform(-self.noise_level, self.noise_level)
            time.sleep(0.05)
        
        print(f"[ArduinoSimulator] 📍 Pozisyon: Pan={self.current_pan:.1f}°, Tilt={self.current_tilt:.1f}°")
        return True
    
    def move_relative(self, pan_delta: float, tilt_delta: float) -> bool:
        """Göreceli hareket"""
        new_pan = self.current_pan + pan_delta
        new_tilt = self.current_tilt + tilt_delta
        return self.move_to_position(new_pan, new_tilt)
    
    def center_position(self) -> bool:
        """Merkez pozisyona git"""
        return self.move_to_position(90, 90)
    
    def fire_laser(self, duration: float = 0.1) -> bool:
        """Lazer ateşleme simulasyonu"""
        if not self.is_connected:
            return False
        
        self.laser_active = True
        print(f"[ArduinoSimulator] 🔴 Lazer ateşlendi ({duration}s)")
        
        # Ateşleme süresi
        time.sleep(duration)
        
        self.laser_active = False
        print("[ArduinoSimulator] ⚫ Lazer kapatıldı")
        return True
    
    def get_position(self) -> Tuple[float, float]:
        """Mevcut pozisyonu döndür"""
        return self.current_pan, self.current_tilt
    
    def is_laser_active(self) -> bool:
        """Lazer aktif mi?"""
        return self.laser_active
    
    def get_status(self) -> dict:
        """Sistem durumu"""
        return {
            'connected': self.is_connected,
            'pan': self.current_pan,
            'tilt': self.current_tilt,
            'laser_active': self.laser_active,
            'port': self.port
        }
    
    def test_movement_pattern(self):
        """Test hareket deseni"""
        print("[ArduinoSimulator] 🔄 Test hareket deseni başlatılıyor...")
        
        # Merkez
        self.center_position()
        time.sleep(1)
        
        # Köşeleri test et
        positions = [
            (45, 45),   # Sol üst
            (135, 45),  # Sağ üst  
            (135, 135), # Sağ alt
            (45, 135),  # Sol alt
            (90, 90)    # Merkez
        ]
        
        for i, (pan, tilt) in enumerate(positions):
            print(f"[ArduinoSimulator] Test pozisyonu {i+1}/5: ({pan}°, {tilt}°)")
            self.move_to_position(pan, tilt)
            time.sleep(0.5)
        
        print("[ArduinoSimulator] ✅ Test hareket deseni tamamlandı")

# Test fonksiyonu
if __name__ == "__main__":
    print("🎮 BARLAS Arduino Simulator Test")
    print("=" * 40)
    
    # Simulator başlat
    arduino_sim = ArduinoSimulator()
    
    print("📋 Mevcut durum:")
    print(arduino_sim.get_status())
    
    print("\n🔄 Test hareket deseni...")
    arduino_sim.test_movement_pattern()
    
    print("\n🔴 Lazer testi...")
    arduino_sim.fire_laser(0.2)
    
    print("\n✅ Test tamamlandı!")
