"""
PID Controller for BARS vehicle
Hesaplanan değerler KTR raporundaki fiziksel parametrelere göre ayarlanmıştır:
- Araç kütlesi: 115 kg
- Eğim açısı: 24.5° 
- Tekerlek yarıçapı: 0.2 m
- Motor maksimum tork: 40 Nm (varsayılan)
"""
import time
import math

class PIDController:
    """
    BARS aracı için özel olarak tasarlanmış PID kontrol sınıfı
    Farklı parkur etapları için farklı parametreler kullanır
    """

    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.05):
        # PID parametreleri (varsayılan değerler)
        self.Kp = Kp  # Proportional gain (varsayılan)
        self.Ki = Ki  # Integral gain (varsayılan)
        self.Kd = Kd  # Derivative gain (varsayılan)
        
        # PID state variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        
        # Sistem fiziksel parametreleri (KTR'den)
        self.m_kg = 115                 # Araç kütlesi (kg)
        self.g = 9.81                   # Yerçekimi ivmesi (m/s²)
        self.theta_deg = 24.5           # Eğim açısı (derece)
        self.r_teker = 0.2              # Tekerlek yarıçapı (m)
        self.T_motor_max = 40           # Motor maksimum tork (Nm) (varsayılan)
        
        # Hesaplanan değerler
        self.sin_theta = math.sin(math.radians(self.theta_deg))
        self.F_eğim = self.m_kg * self.g * self.sin_theta      # 467.73 N
        self.T_teker_toplam = self.F_eğim * self.r_teker       # 93.55 Nm
        self.T_motor_tek = self.T_teker_toplam / 4             # 23.39 Nm per motor
        
        print(f"[PIDController] Başlatıldı - Eğim torku: {self.T_motor_tek:.2f} Nm/motor")

    def set_gains(self, Kp, Ki, Kd):
        """PID parametrelerini günceller"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        print(f"[PIDController] Parametreler güncellendi - Kp:{Kp}, Ki:{Ki}, Kd:{Kd}")

    def reset(self):
        """PID state'ini sıfırlar"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        print("[PIDController] State sıfırlandı")

    def compute(self, setpoint, measured):
        """
        PID kontrol hesaplaması yapar
        setpoint: Hedef değer (tork Nm veya hız m/s)
        measured: Ölçülen değer (encoder/IMU'dan)
        return: Kontrol sinyali (-1.0 ile 1.0 arası)
        """
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0.0:
            dt = 0.01  # Minimum dt (varsayılan)
        
        # Error hesaplama
        error = setpoint - measured
        
        # Integral hesaplama (windup önleme ile)
        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))  # Integral windup önleme
        
        # Derivative hesaplama
        derivative = (error - self.prev_error) / dt
        
        # PID çıkışı
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # State güncelleme
        self.prev_error = error
        self.prev_time = current_time
        
        # Çıkışı normalize et (-1.0 ile 1.0 arası)
        output = max(-1.0, min(1.0, output))
        
        return output

    def compute_pwm(self, setpoint, measured):
        """
        PID hesaplama yapıp doğrudan PWM değeri döndürür
        return: PWM değeri (0-255 arası)
        """
        control_signal = self.compute(setpoint, measured)
        
        # Kontrol sinyalini PWM'e çevir (127 = nötr)
        pwm = int(127 + control_signal * 128)
        pwm = max(0, min(255, pwm))
        
        return pwm

    def get_etap_gains(self, etap):
        """
        Parkur etabına göre önerilen PID parametreleri döndürür
        """
        etap_params = {
            "dik_eğim": {"Kp": 1.5, "Ki": 0.2, "Kd": 0.1},    # Güçlü tepki için
            "hızlanma": {"Kp": 0.8, "Ki": 0.05, "Kd": 0.02},  # Yumuşak hızlanma
            "sığ_su": {"Kp": 1.2, "Ki": 0.15, "Kd": 0.08},    # Kararlı hareket
            "yan_eğim": {"Kp": 1.3, "Ki": 0.18, "Kd": 0.09},  # Denge için
            "taşlı": {"Kp": 1.0, "Ki": 0.12, "Kd": 0.06},     # Titreşim önleme
            "düz": {"Kp": 0.6, "Ki": 0.08, "Kd": 0.03},       # Normal sürüş
            "trafik_konileri": {"Kp": 0.9, "Ki": 0.04, "Kd": 0.05},  # Hassas manevralar
            "engebeli": {"Kp": 1.1, "Ki": 0.14, "Kd": 0.07}   # Engebeli arazi
        }
        
        return etap_params.get(etap, etap_params["düz"])

    def set_etap_params(self, etap):
        """
        Parkur etabına göre PID parametrelerini otomatik ayarlar
        """
        params = self.get_etap_gains(etap)
        self.set_gains(params["Kp"], params["Ki"], params["Kd"])
        self.reset()  # Etap değiştiğinde state'i sıfırla

    def compute_target_torque(self, etap="düz"):
        """
        Parkur etabına göre hedef tork hesaplar
        return: Hedef tork (Nm)
        """
        etap_multipliers = {
            "dik_eğim": 1.0,      # Tam eğim torku
            "hızlanma": 0.7,      # %70 tork
            "sığ_su": 0.8,        # %80 tork (direnç için)
            "yan_eğim": 0.9,      # %90 tork
            "taşlı": 0.85,        # %85 tork
            "düz": 0.6,           # %60 tork
            "trafik_konileri": 0.5,  # %50 tork (yavaş)
            "engebeli": 0.9       # %90 tork
        }
        
        multiplier = etap_multipliers.get(etap, 0.6)
        target_torque = self.T_motor_tek * multiplier
        
        return target_torque

    def get_status(self):
        """
        PID controller durumunu döndürür
        """
        return {
            "Kp": self.Kp,
            "Ki": self.Ki, 
            "Kd": self.Kd,
            "integral": self.integral,
            "prev_error": self.prev_error,
            "target_torque_per_motor": self.T_motor_tek,
            "max_motor_torque": self.T_motor_max
        }


# Test kodu
if __name__ == "__main__":
    print("=== BARS PID Controller Test Başlatılıyor ===")
    
    # PID controller oluştur
    pid = PIDController()
    
    print(f"\n1. Sistem parametreleri:")
    status = pid.get_status()
    for key, value in status.items():
        print(f"   {key}: {value}")
    
    print(f"\n2. Farklı etaplar için hedef torklar:")
    etaplar = ["düz", "dik_eğim", "hızlanma", "sığ_su", "yan_eğim"]
    for etap in etaplar:
        target = pid.compute_target_torque(etap)
        print(f"   {etap}: {target:.2f} Nm")
    
    print(f"\n3. PID hesaplama testi:")
    # Dik eğim etabı için test
    pid.set_etap_params("dik_eğim")
    setpoint = pid.compute_target_torque("dik_eğim")  # 23.39 Nm
    
    # Simüle edilmiş ölçümler
    measurements = [0, 5, 10, 15, 20, 22, 23]
    
    for measured in measurements:
        control_output = pid.compute(setpoint, measured)
        pwm_output = pid.compute_pwm(setpoint, measured)
        error = setpoint - measured
        print(f"   Hedef: {setpoint:.1f}, Ölçülen: {measured:.1f}, Hata: {error:.1f}, "
              f"Kontrol: {control_output:.3f}, PWM: {pwm_output}")
        time.sleep(0.1)
    
    print(f"\n4. Etap değiştirme testi:")
    for etap in ["düz", "dik_eğim", "hızlanma"]:
        pid.set_etap_params(etap)
        params = pid.get_etap_gains(etap)
        print(f"   {etap}: Kp={params['Kp']}, Ki={params['Ki']}, Kd={params['Kd']}")
    
    print("\n=== Test Tamamlandı ===")
