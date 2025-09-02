import serial
import time

class BarlasVehicleController:
    """
    🚗 BARLAS Tam Araç Kontrol Sistemi
    Pan-Tilt + Hareket + Fren + Far + Encoder
    """
    
    def __init__(self, port="/dev/ttyACM0", baud_rate=9600, timeout=2):
        # Serial bağlantı
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        
        # Pan-Tilt sistem durumu
        self.current_pan = 90
        self.current_tilt = 90
        self.laser_active = False
        
        # Hareket sistem durumu
        self.current_speed = 0
        self.direction = "STOP"  # FORWARD, BACKWARD, STOP
        
        # Fren sistemi
        self.brake_active = False
        
        # Far sistemi  
        self.headlight_active = False
        
        # Encoder değerleri
        self.encoder_left = 0
        self.encoder_right = 0
        
        print("[BARLAS] 🚗 Tam araç kontrol sistemi başlatılıyor...")

    def connect(self):
        """Arduino'ya bağlan"""
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            time.sleep(2)  # Arduino reset bekle
            print(f"[BARLAS] ✅ Bağlandı: {self.port}")
            return True
        except Exception as e:
            print(f"[BARLAS] ❌ Bağlantı hatası: {e}")
            return False

    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.ser:
            self.ser.close()
            self.ser = None
            print("[BARLAS] 🔌 Bağlantı kapatıldı")

    def send_command(self, cmd: str):
        """Arduino'ya komut gönder"""
        if self.ser:
            try:
                self.ser.write((cmd + "\n").encode())
                time.sleep(0.05)  # Küçük delay
                print(f"[BARLAS] → {cmd}")
                return True
            except Exception as e:
                print(f"[BARLAS] ❌ Komut hatası: {e}")
                return False
        return False
    
    # ======================================
    # 🎯 PAN-TILT SİSTEMİ
    # ======================================
    
    def move_to_position(self, pan, tilt):
        """Servo'ları belirtilen açıya hareket ettir"""
        self.current_pan = int(pan)
        self.current_tilt = int(tilt)
        return self.send_command(f"MOVE,{self.current_pan},{self.current_tilt}")

    def center_position(self):
        """Pan-Tilt'i merkeze getir"""
        return self.move_to_position(90, 90)

    def enable_laser(self):
        """Lazeri aç"""
        self.laser_active = True
        return self.send_command("LASER,ON")

    def disable_laser(self):
        """Lazeri kapat"""
        self.laser_active = False
        return self.send_command("LASER,OFF")
    
    # ======================================
    # 🚗 HAREKET SİSTEMİ - YENİ GERÇEKÇİ KODLAR
    # ======================================
    
    def move_forward(self, speed=100):
        """İleri hareket (0-255 hız) - BTS7960 motor sürücü"""
        speed = max(0, min(255, speed))
        self.current_speed = speed
        self.direction = "FORWARD"
        success = self.send_command(f"MOTOR,FORWARD,{speed}")
        if success:
            print(f"[BARLAS] 🚗 İleri hareket: Hız {speed}")
        return success
    
    def move_backward(self, speed=100):
        """Geri hareket (0-255 hız) - BTS7960 motor sürücü"""
        speed = max(0, min(255, speed))
        self.current_speed = speed
        self.direction = "BACKWARD"
        success = self.send_command(f"MOTOR,BACKWARD,{speed}")
        if success:
            print(f"[BARLAS] 🔄 Geri hareket: Hız {speed}")
        return success
    
    def turn_left(self, speed=100):
        """Sola dön - Sol motor yavaş, sağ motor hızlı"""
        speed = max(0, min(255, speed))
        self.current_speed = speed
        self.direction = "LEFT"
        success = self.send_command(f"MOTOR,LEFT,{speed}")
        if success:
            print(f"[BARLAS] ◀️ Sola dönüş: Hız {speed}")
        return success
    
    def turn_right(self, speed=100):
        """Sağa dön - Sağ motor yavaş, sol motor hızlı"""
        speed = max(0, min(255, speed))
        self.current_speed = speed
        self.direction = "RIGHT"
        success = self.send_command(f"MOTOR,RIGHT,{speed}")
        if success:
            print(f"[BARLAS] ▶️ Sağa dönüş: Hız {speed}")
        return success
    
    def stop_movement(self):
        """Hareketi durdur - Soft starter ile yavaşça durur"""
        self.current_speed = 0
        self.direction = "STOP"
        success = self.send_command("MOTOR,STOP,0")
        if success:
            print("[BARLAS] ⏹️ Hareket durduruldu")
        return success
    
    # ======================================
    # 🛑 FREN SİSTEMİ - SERVO FREN (GERÇEKÇİ)
    # ======================================
    
    def brake_on(self):
        """Servo freni çek - Fiziksel fren sistemi"""
        self.brake_active = True
        success = self.send_command("BRAKE,ON")
        if success:
            print("[BARLAS] 🛑 FREN ÇEKİLDİ - Servo aktif")
        return success
    
    def brake_off(self):
        """Servo freni bırak - Fren serbest"""
        self.brake_active = False
        success = self.send_command("BRAKE,OFF")
        if success:
            print("[BARLAS] ✅ FREN BIRAKILDI - Servo serbest")
        return success
    
    def emergency_brake(self):
        """ACİL FREN - Motor durdur + Servo fren çek"""
        print("🚨 [BARLAS] ACİL FREN PROSEDÜRÜ!")
        
        # 1. Motorları durdur
        self.stop_movement()
        time.sleep(0.1)
        
        # 2. Servo freni çek
        self.brake_on()
        
        print("�️ [BARLAS] Acil fren tamamlandı!")
        return True
    
    # ======================================
    # 💡 FAR SİSTEMİ - RÖLE KONTROL (GERÇEKÇİ)
    # ======================================
    
    def headlight_on(self):
        """Farları aç - Röle ile kontrol"""
        self.headlight_active = True
        success = self.send_command("FAR,ON")
        if success:
            print("[BARLAS] 💡 FARLAR AÇILDI - Röle aktif")
        return success
    
    def headlight_off(self):
        """Farları kapat - Röle pasif"""
        self.headlight_active = False
        success = self.send_command("FAR,OFF")
        if success:
            print("[BARLAS] 🌙 FARLAR KAPATILDI - Röle pasif")
        return success
    
    def headlight_toggle(self):
        """Far durumunu değiştir"""
        if self.headlight_active:
            return self.headlight_off()
        else:
            return self.headlight_on()
    
    # ======================================
    # 📊 ENCODER SİSTEMİ - GERÇEK ENCODER OKUMA
    # ======================================
    
    def read_encoders(self):
        """
        Encoder değerlerini Arduino'dan oku
        Returns: (left_count, right_count) veya None
        """
        success = self.send_command("ENCODER,READ")
        if success:
            time.sleep(0.1)  # Arduino yanıt için bekle
            
            # Arduino'dan gelen yanıtı parse et
            # Format: "OK - Encoders L:1234,R:5678"
            try:
                if self.ser and self.ser.in_waiting > 0:
                    response = self.ser.readline().decode().strip()
                    if "Encoders" in response:
                        # L:1234,R:5678 kısmını çıkar
                        parts = response.split("L:")
                        if len(parts) > 1:
                            left_part = parts[1].split(",R:")
                            if len(left_part) == 2:
                                self.encoder_left = int(left_part[0])
                                self.encoder_right = int(left_part[1])
                                print(f"[BARLAS] 📊 Encoder Sol:{self.encoder_left}, Sağ:{self.encoder_right}")
                                return (self.encoder_left, self.encoder_right)
            except:
                pass
        
        print("[BARLAS] ⚠️ Encoder okuma hatası")
        return None
    
    def reset_encoders(self):
        """Encoder değerlerini sıfırla"""
        success = self.send_command("ENCODER,RESET")
        if success:
            self.encoder_left = 0
            self.encoder_right = 0
            print("[BARLAS] 🔄 Encoder'lar sıfırlandı")
        return success
    
    def get_distance_traveled(self):
        """
        Encoder'lardan mesafe hesapla (basit hesaplama)
        Tekerlek çapına göre ayarlanmalı
        """
        encoders = self.read_encoders()
        if encoders:
            left, right = encoders
            # Basit hesaplama (tekerlek çapı ve encoder resolution'a göre ayarla)
            wheel_diameter_cm = 10  # Tekerlek çapı (cm) - gerçek değerle değiştir
            encoder_ppr = 360       # Encoder pulse per revolution - gerçek değerle değiştir
            
            wheel_circumference = 3.14159 * wheel_diameter_cm
            left_distance = (left / encoder_ppr) * wheel_circumference
            right_distance = (right / encoder_ppr) * wheel_circumference
            
            avg_distance = (left_distance + right_distance) / 2
            print(f"[BARLAS] 📏 Tahmini mesafe: {avg_distance:.2f} cm")
            return avg_distance
        return 0
    
    # ======================================
    # 🎛️ KONTROL MODU DEĞİŞTİRME
    # ======================================
    
    def set_control_mode(self, mode="SERIAL"):
        """
        Kontrol modunu değiştir
        SERIAL: Python/PC kontrolü
        PIXHAWK: Pixhawk PWM kontrolü
        """
        if mode in ["SERIAL", "PIXHAWK"]:
            success = self.send_command(f"MODE,{mode}")
            if success:
                print(f"[BARLAS] 🎛️ Kontrol modu: {mode}")
            return success
        else:
            print(f"[BARLAS] ❌ Geçersiz mod: {mode}")
            return False
    
    def switch_to_pixhawk(self):
        """Pixhawk kontrol moduna geç"""
        return self.set_control_mode("PIXHAWK")
    
    def switch_to_pc(self):
        """PC kontrol moduna geç"""
        return self.set_control_mode("SERIAL")
    
    # ======================================
    # 🛡️ GÜVENLİK SİSTEMİ
    # ======================================
    
    def get_system_status(self):
        """Sistem durumunu döndür"""
        status = {
            "pan": self.current_pan,
            "tilt": self.current_tilt,
            "laser": self.laser_active,
            "speed": self.current_speed,
            "direction": self.direction,
            "brake": self.brake_active,
            "headlight": self.headlight_active,
            "encoder_left": self.encoder_left,
            "encoder_right": self.encoder_right
        }
        return status
    
    def emergency_stop_all(self):
        """ACİL DURDURMA - Tüm sistemleri güvenli konuma getir"""
        print("🚨 [BARLAS] ACİL DURDURMA PROSEDÜRÜ BAŞLATILIYOR!")
        
        # 1. Hareketi durdur
        self.stop_movement()
        
        # 2. Freni devreye al
        self.brake_on()
        
        # 3. Lazeri kapat
        self.disable_laser()
        
        # 4. Pan-Tilt'i merkeze getir
        self.center_position()
        
        # 5. Durumu rapor et
        print("🛡️ [BARLAS] Tüm sistemler güvenli konuma getirildi!")
        return True


# Geriye uyumluluk için eski class adını koruyalım
class ArduinoPanTiltController(BarlasVehicleController):
    """Eski kod uyumluluğu için"""
    pass
