"""
BARLAS Laser Pan-Tilt Controller
Servo motorları ile lazer pointer kontrolü
Bağımsız çalışabilen modül
"""
import time
import math
from typing import Tuple, Optional

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    print("RPi.GPIO bulunamadı - simülasyon modu")


class LaserPanTiltController:
    """
    Pan-Tilt servo sistemi ile lazer pointer kontrolü
    Kamera görüntüsündeki dart hedefini lazer ile işaretler
    """
    
    def __init__(self, pan_pin=18, tilt_pin=19, laser_pin=20):
        """
        Lazer Pan-Tilt kontrolcüsü
        
        Args:
            pan_pin: Pan servo GPIO pin (varsayılan: 18)
            tilt_pin: Tilt servo GPIO pin (varsayılan: 19) 
            laser_pin: Lazer GPIO pin (varsayılan: 20)
        """
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        self.laser_pin = laser_pin
        
        # Servo pozisyonları (0-180 derece)
        self.pan_position = 90   # Başlangıç merkez
        self.tilt_position = 90  # Başlangıç merkez
        
        # Servo limitleri
        self.pan_min = 10
        self.pan_max = 170
        self.tilt_min = 30
        self.tilt_max = 150
        
        # Lazer durumu
        self.laser_active = False
        
        # PWM nesneleri
        self.pan_pwm = None
        self.tilt_pwm = None
        
        # Kalibrasyon değerleri (kamera-lazer offset)
        self.calibration_offset_x = 0  # Piksel cinsinden X offset
        self.calibration_offset_y = 0  # Piksel cinsinden Y offset
        
        # GPIO kurulumu
        if RPI_AVAILABLE:
            self.setup_gpio()
        else:
            print("[LaserPanTilt] Simülasyon modu - gerçek donanım yok")
        
        print(f"[LaserPanTilt] Başlatıldı - Pan:{self.pan_position}°, Tilt:{self.tilt_position}°")
    
    def setup_gpio(self):
        """GPIO pinlerini ve PWM'i ayarlar"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pan_pin, GPIO.OUT)
            GPIO.setup(self.tilt_pin, GPIO.OUT)
            GPIO.setup(self.laser_pin, GPIO.OUT)
            
            # PWM ayarları (50Hz servo için)
            self.pan_pwm = GPIO.PWM(self.pan_pin, 50)
            self.tilt_pwm = GPIO.PWM(self.tilt_pin, 50)
            
            self.pan_pwm.start(0)
            self.tilt_pwm.start(0)
            
            # Lazer başlangıçta kapalı
            GPIO.output(self.laser_pin, GPIO.LOW)
            
            # Başlangıç pozisyonu
            self.move_to_position(self.pan_position, self.tilt_position)
            
            print("[LaserPanTilt] GPIO kurulumu tamamlandı")
            
        except Exception as e:
            print(f"[LaserPanTilt] GPIO kurulum hatası: {e}")
    
    def angle_to_duty_cycle(self, angle):
        """Servo açısını PWM duty cycle'a çevirir"""
        return 2.5 + (angle / 180.0) * 10.0
    
    def move_to_position(self, pan_angle, tilt_angle):
        """Belirtilen açılara hareket eder"""
        
        # Açı sınırlarını kontrol et
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        self.pan_position = pan_angle
        self.tilt_position = tilt_angle
        
        if RPI_AVAILABLE and self.pan_pwm and self.tilt_pwm:
            try:
                pan_duty = self.angle_to_duty_cycle(pan_angle)
                tilt_duty = self.angle_to_duty_cycle(tilt_angle)
                
                self.pan_pwm.ChangeDutyCycle(pan_duty)
                self.tilt_pwm.ChangeDutyCycle(tilt_duty)
                
                time.sleep(0.2)  # Servo hareket süresi
                
                # PWM'i durdur (servo titremesini önler)
                self.pan_pwm.ChangeDutyCycle(0)
                self.tilt_pwm.ChangeDutyCycle(0)
                
            except Exception as e:
                print(f"[LaserPanTilt] Servo hareket hatası: {e}")
        
        print(f"[LaserPanTilt] Pozisyon: Pan:{pan_angle:.1f}°, Tilt:{tilt_angle:.1f}°")
    
    def enable_laser(self):
        """Lazer pointer'ı açar"""
        if RPI_AVAILABLE:
            try:
                GPIO.output(self.laser_pin, GPIO.HIGH)
                self.laser_active = True
                print("[LaserPanTilt] 🔴 LAZER AKTİF")
            except Exception as e:
                print(f"[LaserPanTilt] Lazer açma hatası: {e}")
        else:
            self.laser_active = True
            print("[LaserPanTilt] 🔴 LAZER AKTİF (Simülasyon)")
    
    def disable_laser(self):
        """Lazer pointer'ı kapatır"""
        if RPI_AVAILABLE:
            try:
                GPIO.output(self.laser_pin, GPIO.LOW)
                self.laser_active = False
                print("[LazerPanTilt] ⚫ Lazer kapatıldı")
            except Exception as e:
                print(f"[LaserPanTilt] Lazer kapatma hatası: {e}")
        else:
            self.laser_active = False
            print("[LaserPanTilt] ⚫ Lazer kapatıldı (Simülasyon)")
    
    def pixel_to_angle(self, pixel_x, pixel_y, frame_width, frame_height):
        """
        Kamera piksel koordinatlarını servo açılarına çevirir
        
        Args:
            pixel_x, pixel_y: Hedef piksel koordinatları
            frame_width, frame_height: Kamera çözünürlüğü
            
        Returns:
            (pan_angle, tilt_angle): Servo açıları
        """
        
        # Merkez noktasından fark
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # Kalibrasyon offset'i uygula
        offset_x = pixel_x - center_x + self.calibration_offset_x
        offset_y = pixel_y - center_y + self.calibration_offset_y
        
        # Piksel farkını açıya çevir (kamera FOV'a göre)
        # Varsayılan kamera FOV: 60° yatay, 45° dikey
        horizontal_fov = 60
        vertical_fov = 45
        
        pan_adjustment = (offset_x / center_x) * (horizontal_fov / 2)
        tilt_adjustment = -(offset_y / center_y) * (vertical_fov / 2)  # Y ekseni ters
        
        # Mevcut pozisyona ekle
        target_pan = self.pan_position + pan_adjustment
        target_tilt = self.tilt_position + tilt_adjustment
        
        return target_pan, target_tilt
    
    def aim_at_pixel(self, pixel_x, pixel_y, frame_width, frame_height):
        """Belirtilen piksel koordinatına lazer ile nişan alır"""
        
        target_pan, target_tilt = self.pixel_to_angle(pixel_x, pixel_y, frame_width, frame_height)
        
        print(f"[LaserPanTilt] Hedef piksel: ({pixel_x}, {pixel_y})")
        print(f"[LaserPanTilt] Servo hedefi: Pan:{target_pan:.1f}°, Tilt:{target_tilt:.1f}°")
        
        self.move_to_position(target_pan, target_tilt)
        
        # Lazer'i aktif et
        self.enable_laser()
    
    def calibrate_offset(self, pixel_offset_x, pixel_offset_y):
        """Kamera-lazer arasındaki offset'i kalibre eder"""
        self.calibration_offset_x = pixel_offset_x
        self.calibration_offset_y = pixel_offset_y
        print(f"[LaserPanTilt] Kalibrasyon güncellendi: X:{pixel_offset_x}, Y:{pixel_offset_y}")
    
    def center_position(self):
        """Lazer'i merkez pozisyona getirir"""
        self.move_to_position(90, 90)
        self.disable_laser()
    
    def cleanup(self):
        """GPIO temizleme"""
        self.disable_laser()
        
        if RPI_AVAILABLE and self.pan_pwm and self.tilt_pwm:
            try:
                self.pan_pwm.stop()
                self.tilt_pwm.stop()
                GPIO.cleanup()
                print("[LaserPanTilt] GPIO temizlendi")
            except Exception as e:
                print(f"[LaserPanTilt] Temizleme hatası: {e}")


def test_laser_controller():
    """Test fonksiyonu - bağımsız çalışabilir"""
    
    print("=" * 50)
    print("🔴 LASER PAN-TILT CONTROLLER TEST")
    print("=" * 50)
    
    try:
        # Controller oluştur
        laser = LaserPanTiltController()
        
        print("\n🎯 Test senaryoları:")
        
        # 1. Merkez pozisyon
        print("\n1. Merkez pozisyon")
        laser.center_position()
        time.sleep(2)
        
        # 2. Köşelere hareket
        positions = [
            (45, 60, "Sol üst"),
            (135, 60, "Sağ üst"), 
            (135, 120, "Sağ alt"),
            (45, 120, "Sol alt"),
            (90, 90, "Merkez")
        ]
        
        for pan, tilt, desc in positions:
            print(f"\n2. {desc}: ({pan}°, {tilt}°)")
            laser.move_to_position(pan, tilt)
            time.sleep(1.5)
        
        # 3. Lazer testi
        print(f"\n3. Lazer testi")
        laser.enable_laser()
        time.sleep(3)
        laser.disable_laser()
        
        # 4. Piksel hedefleme testi
        print(f"\n4. Piksel hedefleme testi (640x480)")
        
        test_pixels = [
            (160, 120, "Sol üst çeyrek"),
            (480, 120, "Sağ üst çeyrek"),
            (320, 240, "Merkez"),
            (480, 360, "Sağ alt çeyrek"),
            (160, 360, "Sol alt çeyrek")
        ]
        
        for px, py, desc in test_pixels:
            print(f"   → {desc}: piksel ({px}, {py})")
            laser.aim_at_pixel(px, py, 640, 480)
            time.sleep(2)
            laser.disable_laser()
            time.sleep(1)
        
        print(f"\n✅ Test tamamlandı!")
        
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if 'laser' in locals():
            laser.cleanup()


if __name__ == "__main__":
    test_laser_controller()
