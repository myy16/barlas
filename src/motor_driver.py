"""
Motor Driver for BARS 6-wheel vehicle
4 traction motors (left_front, left_rear, right_front, right_rear)  
2 brake servos (fren_left, fren_right) on middle wheels
"""
import time

try:
    import pigpio  # Raspberry Pi için PWM kütüphanesi
except ImportError:
    pigpio = None
    print("[MotorDriver] Uyarı: pigpio yüklenemedi. Donanım PWM çalışmaz, sadece test modunda!")

class MotorDriver:
    """
    4 çekiş motoru ve 2 fren servo motoru için sürücü.
    KTR raporuna uygun 6 tekerlekli sistem:
    - 4 motor tahrikli teker (skid-steering)
    - 2 orta teker servo kontrollü mekanik frenli
    """

    def __init__(self):
        # GPIO pin numaraları (varsayılan değerler)
        self.pins = {
            "left_front": 17,      # Sol ön çekiş motoru
            "left_rear": 18,       # Sol arka çekiş motoru  
            "right_front": 22,     # Sağ ön çekiş motoru
            "right_rear": 23,      # Sağ arka çekiş motoru
            "fren_left": 24,       # Sol orta teker fren servo
            "fren_right": 25       # Sağ orta teker fren servo
        }

        # pigpio bağlantısı
        if pigpio:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon çalışmıyor! 'sudo pigpiod' ile başlatın.")
        else:
            self.pi = None
            print("[MotorDriver] Test modu: pigpio yok, sadece konsol çıktısı")

        # Başlangıçta tüm motorları nötr yap
        for motor in ["left_front", "left_rear", "right_front", "right_rear"]:
            self.set_pwm(motor, 127)

        # Başlangıçta fren serbest bırak
        self.release_fren()
        print("[MotorDriver] Başlatıldı - 4 motor nötr, frenler serbest")

    def pwm_to_us(self, pwm_val):
        """
        PWM değerini microsecond pulse width'e çevirir
        127 = nötr (1.5ms), 0-255 → 1-2ms arası
        """
        pwm_val = max(0, min(255, pwm_val))
        return 1000 + int((pwm_val / 255) * 1000)

    def set_pwm(self, motor, pwm_val):
        """
        Belirtilen motora PWM sinyali gönderir
        motor: "left_front", "left_rear", "right_front", "right_rear"
        pwm_val: 0-255 arası (127=nötr, >127=ileri, <127=geri)
        """
        if motor not in ["left_front", "left_rear", "right_front", "right_rear"]:
            raise ValueError(f"Geçersiz motor adı: {motor}. Geçerli değerler: left_front, left_rear, right_front, right_rear")
        
        pwm_val = max(0, min(255, pwm_val))  # 0-255 arası sınırla
        pulse = self.pwm_to_us(pwm_val)
        
        if self.pi:
            self.pi.set_servo_pulsewidth(self.pins[motor], pulse)
        
        print(f"[MotorDriver] {motor} → PWM {pwm_val} ({pulse}μs)")

    def apply_fren(self):
        """
        Orta tekerlerdeki fren servolarını çeker (mekanik fren aktif)
        Eğimde durma ve acil durumlarda kullanılır
        """
        for fren in ["fren_left", "fren_right"]:
            if self.pi:
                self.pi.set_servo_pulsewidth(self.pins[fren], 2000)  # Fren çekili pozisyon
            print(f"[MotorDriver] {fren} → FREN ÇEKİLDİ")
        print("[MotorDriver] Mekanik frenler aktif - araç sabitlendi")

    def release_fren(self):
        """
        Orta tekerlerdeki fren servolarını bırakır (fren serbest)
        Normal hareket için gerekli
        """
        for fren in ["fren_left", "fren_right"]:
            if self.pi:
                self.pi.set_servo_pulsewidth(self.pins[fren], 1000)  # Fren serbest pozisyon
            print(f"[MotorDriver] {fren} → FREN SERBEST")
        print("[MotorDriver] Mekanik frenler serbest - hareket hazır")

    def stop_all(self):
        """
        Tüm motorları durdur ve acil fren uygula
        Güvenlik fonksiyonu - acil durumlarda kullanılır
        """
        print("[MotorDriver] ACİL DURDURMA başlatılıyor...")
        
        # Tüm motorları nötr yap
        for motor in ["left_front", "left_rear", "right_front", "right_rear"]:
            self.set_pwm(motor, 127)
        
        # Fren uygula
        self.apply_fren()
        print("[MotorDriver] Tüm motorlar durduruldu ve fren çekildi")

    def set_vehicle_motion(self, linear_speed, angular_speed):
        """
        Araç hareket komutunu 4 motora PWM olarak dağıtır
        linear_speed: ileri/geri hız (-1.0 ile 1.0 arası)
        angular_speed: dönüş hızı (-1.0 ile 1.0 arası, + saat yönü)
        """
        # Sol ve sağ taraf PWM oranlarını hesapla
        left_ratio = linear_speed - angular_speed
        right_ratio = linear_speed + angular_speed
        
        # Normalize et
        max_ratio = max(abs(left_ratio), abs(right_ratio), 1.0)
        left_ratio /= max_ratio
        right_ratio /= max_ratio
        
        # PWM değerlerine çevir (127 = nötr)
        left_pwm = int(127 + left_ratio * 128)
        right_pwm = int(127 + right_ratio * 128)
        
        # Motorlara uygula
        self.set_pwm("left_front", left_pwm)
        self.set_pwm("left_rear", left_pwm)
        self.set_pwm("right_front", right_pwm)
        self.set_pwm("right_rear", right_pwm)

    def cleanup(self):
        """
        Sistem kapatma - tüm motorları durdur ve pigpio'yu kapat
        """
        print("[MotorDriver] Sistem kapatılıyor...")
        self.stop_all()
        
        if self.pi:
            # Tüm PWM sinyallerini kapat
            for pin in self.pins.values():
                self.pi.set_servo_pulsewidth(pin, 0)
            self.pi.stop()
        
        print("[MotorDriver] Sistem güvenli şekilde kapatıldı")

    def get_status(self):
        """
        Motor sürücü durumunu döndürür
        """
        return {
            "pigpio_connected": self.pi.connected if self.pi else False,
            "pins": self.pins,
            "total_motors": 4,
            "brake_servos": 2
        }


# Test kodu
if __name__ == "__main__":
    print("=== BARS Motor Driver Test Başlatılıyor ===")
    
    try:
        # MotorDriver sınıfını başlat
        driver = MotorDriver()
        
        print("\n1. Tüm motorları nötr konuma getiriliyor...")
        driver.set_pwm("left_front", 127)
        driver.set_pwm("left_rear", 127)
        driver.set_pwm("right_front", 127)
        driver.set_pwm("right_rear", 127)
        time.sleep(1)

        print("\n2. Sol taraf ileri, sağ taraf geri (dönüş testi)...")
        driver.set_pwm("left_front", 200)
        driver.set_pwm("left_rear", 200)
        driver.set_pwm("right_front", 50)
        driver.set_pwm("right_rear", 50)
        time.sleep(2)

        print("\n3. Araç hareket komutu testi...")
        driver.set_vehicle_motion(0.5, 0.0)  # %50 ileri
        time.sleep(1)
        driver.set_vehicle_motion(0.0, 0.3)  # Saat yönünde dönüş
        time.sleep(1)

        print("\n4. Fren sistemi testi...")
        driver.apply_fren()
        time.sleep(1)
        driver.release_fren()
        time.sleep(1)

        print("\n5. Acil durdurma testi...")
        driver.stop_all()
        time.sleep(1)

    except Exception as e:
        print(f"HATA: {e}")
    
    finally:
        if 'driver' in locals():
            driver.cleanup()
        print("\n=== Test Tamamlandı ===")
