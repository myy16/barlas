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
        self.movement_speed = 2.0     # derece/saniye (YAVAŞLATILDI)
        self.noise_level = 0.5        # pozisyon gürültüsü
        
        # TARAMA SİSTEMİ
        self.scanning_enabled = True
        self.scanning_thread = None
        self.scan_speed = 5.0         # derece/saniye tarama hızı
        self.scan_step = 2.0          # her adımda kaç derece
        self.scan_delay = 0.4         # adımlar arası bekleme (saniye)
        
        # Tarama durumu
        self.is_scanning = False
        self.scan_direction_pan = 1   # 1 = sağa, -1 = sola
        self.scan_direction_tilt = 1  # 1 = aşağı, -1 = yukarı
        self.target_locked = False    # Hedef kilitli mi?
        
        print(f"[ArduinoSimulator] 🎮 Simulator başlatıldı")
        print(f"[ArduinoSimulator] Pan: {self.current_pan}°, Tilt: {self.current_tilt}°")
        print(f"[ArduinoSimulator] 🔄 Tarama modu: {'AÇIK' if self.scanning_enabled else 'KAPALI'}")
        
        # Tarama başlat
        if self.scanning_enabled:
            self.start_scanning()
    
    def connect(self) -> bool:
        """Simülasyon - her zaman başarılı"""
        print("[ArduinoSimulator] ✅ Simülasyon bağlantısı sağlandı")
        time.sleep(0.5)  # Gerçek bağlantıyı taklit et
        return True
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        self.is_connected = False
        self.stop_scanning()
        print("[ArduinoSimulator] 🔌 Simülasyon bağlantısı kapatıldı")
    
    def start_scanning(self):
        """Tarama modunu başlat"""
        if self.scanning_thread is None or not self.scanning_thread.is_alive():
            self.is_scanning = True
            self.scanning_thread = threading.Thread(target=self._scanning_loop, daemon=True)
            self.scanning_thread.start()
            print("[ArduinoSimulator] 🔄 Tarama modu başlatıldı")
    
    def stop_scanning(self):
        """Tarama modunu durdur"""
        self.is_scanning = False
        if self.scanning_thread:
            self.scanning_thread.join(timeout=1.0)
        print("[ArduinoSimulator] ⏹️ Tarama modu durduruldu")
    
    def pause_scanning(self):
        """Tarama geçici olarak durdur (hedef kilitlendiğinde)"""
        self.target_locked = True
        print("[ArduinoSimulator] ⏸️ Tarama duraklatıldı - hedef kilitli")
    
    def resume_scanning(self):
        """Tarama devam et (hedef kaybedildiğinde)"""
        self.target_locked = False
        print("[ArduinoSimulator] ▶️ Tarama devam ediyor")
    
    def _scanning_loop(self):
        """Tarama döngüsü (ayrı thread'de çalışır)"""
        while self.is_scanning and self.is_connected:
            try:
                # Hedef kilitliyse tarama yapma
                if self.target_locked:
                    time.sleep(0.1)
                    continue
                
                # Pan tarama (yatay - sağdan sola)
                new_pan = self.current_pan + (self.scan_step * self.scan_direction_pan)
                
                # Pan sınırları kontrol et ve yön değiştir
                if new_pan >= self.pan_max:
                    new_pan = self.pan_max
                    self.scan_direction_pan = -1  # sola dön
                    # Tilt de bir adım hareket et
                    new_tilt = self.current_tilt + (self.scan_step * self.scan_direction_tilt)
                    if new_tilt >= self.tilt_max:
                        new_tilt = self.tilt_max
                        self.scan_direction_tilt = -1  # yukarı dön
                    elif new_tilt <= self.tilt_min:
                        new_tilt = self.tilt_min
                        self.scan_direction_tilt = 1   # aşağı dön
                    
                    self.current_tilt = new_tilt
                    
                elif new_pan <= self.pan_min:
                    new_pan = self.pan_min
                    self.scan_direction_pan = 1   # sağa dön
                    # Tilt de bir adım hareket et
                    new_tilt = self.current_tilt + (self.scan_step * self.scan_direction_tilt)
                    if new_tilt >= self.tilt_max:
                        new_tilt = self.tilt_max
                        self.scan_direction_tilt = -1  # yukarı dön
                    elif new_tilt <= self.tilt_min:
                        new_tilt = self.tilt_min
                        self.scan_direction_tilt = 1   # aşağı dön
                    
                    self.current_tilt = new_tilt
                
                self.current_pan = new_pan
                
                # Gürültü ekle
                noise_pan = random.uniform(-0.2, 0.2)
                noise_tilt = random.uniform(-0.2, 0.2)
                self.current_pan += noise_pan
                self.current_tilt += noise_tilt
                
                # Her 20 adımda bir pozisyon bildir
                if random.randint(1, 20) == 1:
                    print(f"[ArduinoSimulator] 🔄 Tarama: Pan={self.current_pan:.1f}°, Tilt={self.current_tilt:.1f}°")
                
                # Tarama hızını kontrol et
                time.sleep(self.scan_delay)
                
            except Exception as e:
                print(f"[ArduinoSimulator] Tarama hatası: {e}")
                time.sleep(1.0)
    
    def move_to_position(self, pan: float, tilt: float, smooth: bool = True) -> bool:
        """Belirtilen pozisyona hareket et"""
        if not self.is_connected:
            return False
        
        # Hedef kilitlendiğinde taramayı durdur
        self.pause_scanning()
        
        # Limitleri kontrol et
        pan = max(self.pan_min, min(self.pan_max, pan))
        tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        
        old_pan, old_tilt = self.current_pan, self.current_tilt
        
        if smooth:
            # Yumuşak hareket simulasyonu (daha yavaş)
            pan_diff = pan - self.current_pan
            tilt_diff = tilt - self.current_tilt
            
            # Hareket süresi hesapla (daha yavaş)
            max_movement = max(abs(pan_diff), abs(tilt_diff))
            movement_time = max_movement / self.movement_speed
            
            # Aşamalı hareket (daha detaylı adımlar)
            steps = max(1, int(movement_time * 5))  # 200ms adımlar
            
            for step in range(steps + 1):
                progress = step / steps
                
                # Yumuşak interpolasyon
                current_pan = old_pan + pan_diff * progress
                current_tilt = old_tilt + tilt_diff * progress
                
                # Gürültü ekle (daha az)
                noise_pan = random.uniform(-self.noise_level * 0.5, self.noise_level * 0.5)
                noise_tilt = random.uniform(-self.noise_level * 0.5, self.noise_level * 0.5)
                
                self.current_pan = current_pan + noise_pan
                self.current_tilt = current_tilt + noise_tilt
                
                time.sleep(0.2)  # Yavaş hareket
            
        else:
            # Anında hareket (yine de biraz gürültü)
            self.current_pan = pan + random.uniform(-self.noise_level, self.noise_level)
            self.current_tilt = tilt + random.uniform(-self.noise_level, self.noise_level)
            time.sleep(0.1)
        
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
    
    def enable_laser(self) -> bool:
        """Lazer açma - kalıcı"""
        if not self.is_connected:
            return False
        
        self.laser_active = True
        print("[ArduinoSimulator] 🔴 LAZER AÇILDI")
        return True
    
    def disable_laser(self) -> bool:
        """Lazer kapama"""
        if not self.is_connected:
            return False
        
        self.laser_active = False
        print("[ArduinoSimulator] ⚫ LAZER KAPATILDI")
        # Lazer kapandıktan sonra taramaya devam et
        self.resume_scanning()
        return True
    
    def fire_laser(self, duration: float = 0.1) -> bool:
        """Lazer ateşleme simulasyonu (kısa süreli)"""
        if not self.is_connected:
            return False
        
        self.laser_active = True
        print(f"[ArduinoSimulator] 🔴 Lazer ateşlendi ({duration}s)")
        
        # Ateşleme süresi
        time.sleep(duration)
        
        self.laser_active = False
        print("[ArduinoSimulator] ⚫ Lazer kapatıldı")
        self.resume_scanning()
        return True
    
    def pixel_to_angle(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> Tuple[float, float]:
        """Piksel koordinatlarını servo açılarına çevir"""
        # Kamera merkezi
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # Offset hesapla
        offset_x = pixel_x - center_x
        offset_y = pixel_y - center_y
        
        # Kamera FOV (60° yatay, 45° dikey)
        horizontal_fov = 60
        vertical_fov = 45
        
        # Açı hesapla
        pan_adjustment = (offset_x / center_x) * (horizontal_fov / 2)
        tilt_adjustment = -(offset_y / center_y) * (vertical_fov / 2)
        
        # Mevcut pozisyona ekle
        target_pan = self.current_pan + pan_adjustment
        target_tilt = self.current_tilt + tilt_adjustment
        
        # Sınırları kontrol et
        target_pan = max(self.pan_min, min(self.pan_max, target_pan))
        target_tilt = max(self.tilt_min, min(self.tilt_max, target_tilt))
        
        return target_pan, target_tilt
    
    def aim_at_pixel(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> bool:
        """Piksele nişan al (eski uyumluluk için)"""
        target_pan, target_tilt = self.pixel_to_angle(pixel_x, pixel_y, frame_width, frame_height)
        
        print(f"[ArduinoSimulator] 🎯 HEDEF: Piksel ({pixel_x}, {pixel_y}) -> Servo ({target_pan:.1f}°, {target_tilt:.1f}°)")
        
        # Hedef pozisyona git (taramayı durdur)
        success = self.move_to_position(target_pan, target_tilt)
        if success:
            self.enable_laser()
        return success
    
    def get_position(self) -> Tuple[float, float]:
        """Mevcut pozisyonu döndür"""
        return self.current_pan, self.current_tilt
    
    def is_laser_active(self) -> bool:
        """Lazer aktif mi?"""
        return self.laser_active
    
    def set_scanning_speed(self, speed: float):
        """Tarama hızını ayarla (derece/saniye)"""
        self.scan_speed = speed
        self.scan_delay = 1.0 / speed  # Hız ayarı
        print(f"[ArduinoSimulator] ⚙️ Tarama hızı: {speed} derece/saniye")
    
    def get_status(self) -> dict:
        """Sistem durumu"""
        return {
            'connected': self.is_connected,
            'pan': self.current_pan,
            'tilt': self.current_tilt,
            'laser_active': self.laser_active,
            'scanning': self.is_scanning,
            'target_locked': self.target_locked,
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
