"""
BARLAS Pan-Tilt Servo Controller
Kamera yönlendirme sistemi - dart hedefleme için
"""
import time
try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    print("RPi.GPIO bulunamadı - simülasyon modu")

class PanTiltController:
    def __init__(self, pan_pin=18, tilt_pin=19):
        """
        Pan-Tilt servo kontrolcüsü
        
        Args:
            pan_pin: Pan servo GPIO pin (varsayılan: 18)
            tilt_pin: Tilt servo GPIO pin (varsayılan: 19)
        """
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        
        # Servo pozisyonları (0-180 derece)
        self.pan_position = 90   # Başlangıç merkez
        self.tilt_position = 90  # Başlangıç merkez
        
        # Servo limitleri
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 30
        self.tilt_max = 150
        
        # PWM nesneleri
        self.pan_pwm = None
        self.tilt_pwm = None
        
        # GPIO kurulumu
        if RPI_AVAILABLE:
            self.setup_gpio()
        else:
            print("[PanTiltController] Simülasyon modu - gerçek servo yok")
        
        print(f"[PanTiltController] Başlatıldı - Pan:{self.pan_position}°, Tilt:{self.tilt_position}°")
    
    def setup_gpio(self):
        """GPIO pinlerini ve PWM'i ayarlar"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pan_pin, GPIO.OUT)
            GPIO.setup(self.tilt_pin, GPIO.OUT)
            
            # PWM ayarları (50Hz servo için)
            self.pan_pwm = GPIO.PWM(self.pan_pin, 50)
            self.tilt_pwm = GPIO.PWM(self.tilt_pin, 50)
            
            self.pan_pwm.start(0)
            self.tilt_pwm.start(0)
            
            # Başlangıç pozisyonu
            self.move_to_position(self.pan_position, self.tilt_position)
            
        except Exception as e:
            print(f"[PanTiltController] GPIO kurulum hatası: {e}")
    
    def angle_to_duty_cycle(self, angle):
        """Servo açısını PWM duty cycle'a çevirir"""
        # 0° = 2.5% duty cycle, 180° = 12.5% duty cycle
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
                
                time.sleep(0.1)  # Servo hareket süresi
                
                # PWM'i durdur (servo titremesini önler)
                self.pan_pwm.ChangeDutyCycle(0)
                self.tilt_pwm.ChangeDutyCycle(0)
                
            except Exception as e:
                print(f"[PanTiltController] Servo hareket hatası: {e}")
        
        print(f"[PanTiltController] Pozisyon: Pan:{pan_angle}°, Tilt:{tilt_angle}°")
    
    def adjust_pan(self, adjustment):
        """Pan pozisyonunu ayarlar (göreceli hareket)"""
        new_pan = self.pan_position + adjustment
        self.move_to_position(new_pan, self.tilt_position)
    
    def adjust_tilt(self, adjustment):
        """Tilt pozisyonunu ayarlar (göreceli hareket)"""
        new_tilt = self.tilt_position + adjustment
        self.move_to_position(self.pan_position, new_tilt)
    
    def center_camera(self):
        """Kamerayı merkez pozisyona getirir"""
        self.move_to_position(90, 90)
    
    def scan_area(self, scan_range=30, steps=5):
        """Belirli bir alanı tarar (dart arama için)"""
        print("[PanTiltController] Alan tarama başlatılıyor...")
        
        original_pan = self.pan_position
        original_tilt = self.tilt_position
        
        # Yatay tarama
        for i in range(steps):
            pan_angle = original_pan - scan_range + (2 * scan_range * i / (steps - 1))
            self.move_to_position(pan_angle, original_tilt)
            time.sleep(0.5)
        
        # Merkeze dön
        self.move_to_position(original_pan, original_tilt)
        print("[PanTiltController] Alan tarama tamamlandı")
    
    def get_position(self):
        """Mevcut pan-tilt pozisyonunu döndürür"""
        return (self.pan_position, self.tilt_position)
    
    def cleanup(self):
        """GPIO temizleme"""
        if RPI_AVAILABLE and self.pan_pwm and self.tilt_pwm:
            try:
                self.pan_pwm.stop()
                self.tilt_pwm.stop()
                GPIO.cleanup()
                print("[PanTiltController] GPIO temizlendi")
            except Exception as e:
                print(f"[PanTiltController] Temizleme hatası: {e}")

def move_pan_tilt(x, y):
    """Eski uyumluluk fonksiyonu"""
    print(f"Moving pan-tilt to X:{x}, Y:{y}")

# Test fonksiyonu
if __name__ == "__main__":
    print("=== Pan-Tilt Controller Test ===")
    
    try:
        controller = PanTiltController()
        
        print("\n1. Merkez pozisyon")
        controller.center_camera()
        time.sleep(1)
        
        print("\n2. Pozisyon testi")
        test_positions = [
            (45, 60),   # Sol-aşağı
            (135, 60),  # Sağ-aşağı
            (135, 120), # Sağ-yukarı
            (45, 120),  # Sol-yukarı
            (90, 90)    # Merkez
        ]
        
        for pan, tilt in test_positions:
            print(f"   Hareket: ({pan}°, {tilt}°)")
            controller.move_to_position(pan, tilt)
            time.sleep(1)
        
        print("\n3. Göreceli hareket testi")
        controller.adjust_pan(20)
        time.sleep(0.5)
        controller.adjust_tilt(-10)
        time.sleep(0.5)
        
        print("\n4. Alan tarama testi")
        controller.scan_area(scan_range=20, steps=3)
        
        print("\n5. Mevcut pozisyon:")
        pan, tilt = controller.get_position()
        print(f"   Pan: {pan}°, Tilt: {tilt}°")
        
    except KeyboardInterrupt:
        print("\nTest iptal edildi")
    
    finally:
        if 'controller' in locals():
            controller.cleanup()
        print("Test tamamlandı")
