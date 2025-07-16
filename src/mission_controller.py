"""
Mission Controller for BARS vehicle
Ana kontrol merkezi - tüm sistemi koordine eder
- Tabela tanıma sonuçlarını alır
- Parkur etabına göre hedefler belirler
- PID controller'ı yönetir
- Motor driver'a komutlar gönderir
- Güvenlik kontrollerini yapar
"""
import time
from motor_driver import MotorDriver
from pid_controller import PIDController

class MissionController:
    """
    BARS aracının ana kontrol sistemi
    Tabela tanıma → Etap belirleme → PID ayarlama → Motor kontrol
    """
    
    def __init__(self):
        print("[MissionController] Sistem başlatılıyor...")
        
        # Alt sistem bileşenlerini başlat
        try:
            self.motor_driver = MotorDriver()
            self.pid_controller = PIDController()
            print("[MissionController] Alt sistemler başarıyla başlatıldı")
        except Exception as e:
            print(f"[MissionController] HATA: Alt sistem başlatılamadı - {e}")
            raise
        
        # Sistem durumu
        self.current_etap = "düz"           # Başlangıç etabı
        self.current_target_torque = 0.0    # Hedef tork
        self.current_measured_torque = 0.0  # Ölçülen tork (encoder'dan gelecek)
        self.is_running = False             # Sistem çalışıyor mu?
        self.emergency_stop = False         # Acil durdurma durumu
        
        # Sistem performans metrikleri
        self.total_distance = 0.0           # Toplam kat edilen mesafe
        self.mission_start_time = 0.0       # Görev başlangıç zamanı
        self.etap_completion_times = {}     # Her etap için tamamlanma süreleri
        
        print("[MissionController] Başlatıldı - Sistem hazır")

    def start_mission(self):
        """
        Ana görevi başlatır
        """
        print("[MissionController] GÖREV BAŞLATILIYOR...")
        self.is_running = True
        self.mission_start_time = time.time()
        self.motor_driver.release_fren()  # Fren serbest bırak
        print("[MissionController] Görev aktif - hareket başlayabilir")

    def stop_mission(self):
        """
        Ana görevi durdurur
        """
        print("[MissionController] GÖREV DURDURULUYOR...")
        self.is_running = False
        self.motor_driver.stop_all()  # Tüm motorları durdur ve fren çek
        
        # Performans raporu
        total_time = time.time() - self.mission_start_time
        print(f"[MissionController] Görev tamamlandı - Toplam süre: {total_time:.1f}s")

    def emergency_stop_activated(self):
        """
        Acil durdurma prosedürü
        """
        print("[MissionController] ⚠️  ACİL DURDURMA AKTİF ⚠️")
        self.emergency_stop = True
        self.is_running = False
        self.motor_driver.stop_all()
        print("[MissionController] Sistem güvenli duruma getirildi")

    def update_etap(self, new_etap):
        """
        Yeni etap bilgisi geldiğinde çağrılır (tabela tanıma'dan)
        """
        if new_etap != self.current_etap:
            etap_start_time = time.time()
            print(f"[MissionController] Etap değişti: {self.current_etap} → {new_etap}")
            
            # Önceki etabın tamamlanma süresini kaydet
            if hasattr(self, 'current_etap_start_time'):
                completion_time = etap_start_time - self.current_etap_start_time
                self.etap_completion_times[self.current_etap] = completion_time
                print(f"[MissionController] {self.current_etap} etabı {completion_time:.1f}s'de tamamlandı")
            
            # Yeni etap ayarları
            self.current_etap = new_etap
            self.current_etap_start_time = etap_start_time
            
            # PID parametrelerini yeni etaba göre ayarla
            self.pid_controller.set_etap_params(new_etap)
            
            # Yeni hedef torku hesapla
            self.current_target_torque = self.pid_controller.compute_target_torque(new_etap)
            print(f"[MissionController] Yeni hedef tork: {self.current_target_torque:.2f} Nm")
            
            # Özel etap kontrolleri
            self._handle_special_etap_requirements(new_etap)

    def _handle_special_etap_requirements(self, etap):
        """
        Özel etap gereksinimleri (fren, hız limitleri vs.)
        """
        if etap == "dik_eğim":
            print("[MissionController] Dik eğim modu - fren sistemi hazır")
            # Dik eğimde ekstra kontrol gerekebilir
            
        elif etap == "sığ_su":
            print("[MissionController] Su geçiş modu - yavaş ve kararlı hareket")
            # Su geçişinde özel hız profili
            
        elif etap == "trafik_konileri":
            print("[MissionController] Slalom modu - hassas kontrol aktif")
            # Daha hassas PID kontrol gerekebilir
            
        elif etap == "hızlanma":
            print("[MissionController] Hızlanma modu - maksimum performans")
            # Fren serbest, tam güç
            self.motor_driver.release_fren()

    def update_sensor_data(self, measured_torque=None, measured_speed=None, imu_data=None):
        """
        Sensör verilerini günceller (encoder, IMU vs.)
        """
        if measured_torque is not None:
            self.current_measured_torque = measured_torque
        
        # IMU verisiyle eğim kontrolü
        if imu_data and 'eğim' in imu_data:
            eğim = imu_data['eğim']
            if abs(eğim) > 20:  # 20 dereceden fazla eğim
                print(f"[MissionController] Yüksek eğim algılandı: {eğim:.1f}°")
                # Otomatik fren devreye girebilir

    def control_loop(self):
        """
        Ana kontrol döngüsü - sürekli çağrılması gereken fonksiyon
        """
        if not self.is_running or self.emergency_stop:
            return
        
        # PID hesaplama
        control_output = self.pid_controller.compute(
            self.current_target_torque, 
            self.current_measured_torque
        )
        
        # PWM değerlerini hesapla
        base_pwm = int(127 + control_output * 128)
        base_pwm = max(0, min(255, base_pwm))
        
        # Motorlara PWM gönder (şimdilik tüm motorlara aynı değer)
        # Gelecekte dönüş kontrol eklendiğinde farklılaştırılabilir
        self.motor_driver.set_pwm("left_front", base_pwm)
        self.motor_driver.set_pwm("left_rear", base_pwm)
        self.motor_driver.set_pwm("right_front", base_pwm)
        self.motor_driver.set_pwm("right_rear", base_pwm)

    def set_vehicle_motion(self, linear_speed, angular_speed):
        """
        Direkt araç hareket komutu (joystick/otonom kontrol için)
        linear_speed: -1.0 ile 1.0 arası (ileri/geri)
        angular_speed: -1.0 ile 1.0 arası (sol/sağ)
        """
        if not self.is_running or self.emergency_stop:
            print("[MissionController] Hareket komutu reddedildi - sistem durmuş")
            return
        
        self.motor_driver.set_vehicle_motion(linear_speed, angular_speed)
        print(f"[MissionController] Hareket komutu: L={linear_speed:.2f}, A={angular_speed:.2f}")

    def get_system_status(self):
        """
        Sistem durumu raporu
        """
        return {
            "is_running": self.is_running,
            "emergency_stop": self.emergency_stop,
            "current_etap": self.current_etap,
            "target_torque": self.current_target_torque,
            "measured_torque": self.current_measured_torque,
            "mission_time": time.time() - self.mission_start_time if self.mission_start_time > 0 else 0,
            "motor_status": self.motor_driver.get_status(),
            "pid_status": self.pid_controller.get_status()
        }

    def cleanup(self):
        """
        Sistem kapatma
        """
        print("[MissionController] Sistem kapatılıyor...")
        self.stop_mission()
        self.motor_driver.cleanup()
        print("[MissionController] Sistem güvenli şekilde kapatıldı")


# Test kodu
if __name__ == "__main__":
    print("=== BARS Mission Controller Test Başlatılıyor ===")
    
    try:
        # Mission Controller başlat
        mission = MissionController()
        
        print("\n1. Sistem durumu:")
        status = mission.get_system_status()
        for key, value in status.items():
            if isinstance(value, dict):
                print(f"   {key}: {list(value.keys())}")
            else:
                print(f"   {key}: {value}")
        
        print("\n2. Görev başlatma:")
        mission.start_mission()
        
        print("\n3. Etap değişiklikleri testi:")
        test_etaplar = ["düz", "dik_eğim", "hızlanma", "sığ_su", "trafik_konileri"]
        
        for etap in test_etaplar:
            print(f"\n--- {etap.upper()} ETABI ---")
            mission.update_etap(etap)
            
            # Simüle edilmiş sensör verisi
            simulated_torque = mission.current_target_torque * 0.8  # %80 yaklaşım
            mission.update_sensor_data(measured_torque=simulated_torque)
            
            # Kontrol döngüsünü çalıştır
            mission.control_loop()
            
            time.sleep(0.5)  # Kısa bekleme
        
        print("\n4. Hareket kontrol testi:")
        mission.set_vehicle_motion(0.5, 0.0)    # İleri
        time.sleep(1)
        mission.set_vehicle_motion(0.0, 0.3)    # Sağa dön
        time.sleep(1)
        mission.set_vehicle_motion(-0.3, 0.0)   # Geri
        time.sleep(1)
        
        print("\n5. Acil durdurma testi:")
        mission.emergency_stop_activated()
        
        # Sistem durumunu kontrol et
        final_status = mission.get_system_status()
        print(f"\nFinal durum - Çalışıyor: {final_status['is_running']}, "
              f"Acil stop: {final_status['emergency_stop']}")
        
    except Exception as e:
        print(f"HATA: {e}")
    
    finally:
        if 'mission' in locals():
            mission.cleanup()
        print("\n=== Test Tamamlandı ===")