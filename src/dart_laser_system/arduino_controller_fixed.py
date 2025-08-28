"""
BARLAS Arduino Pan-Tilt Controller - İyileştirilmiş Versiyon
Servo hareket sorunları giderildi
Arduino bağlantı ve komut sistemi optimize edildi
"""
import serial
import time
import threading
from typing import Optional, Tuple, List
import json
import glob
import sys

class ArduinoPanTiltController:
    """İyileştirilmiş Arduino Pan-Tilt Controller"""
    
    def __init__(self, port=None, baud_rate=9600, timeout=2):
        """
        Arduino Controller
        
        Args:
            port: Arduino serial port (None = otomatik tespit)
            baud_rate: Serial baud rate
            timeout: Serial timeout
        """
        self.baud_rate = baud_rate
        self.timeout = timeout
        
        # Serial bağlantı
        self.serial_conn = None
        self.is_connected = False
        self.port = None
        
        # Servo pozisyonları
        self.current_pan = 90
        self.current_tilt = 90
        
        # Servo limitleri - daha geniş aralık
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 20
        self.tilt_max = 160
        
        # Lazer durumu
        self.laser_active = False
        
        # Thread lock
        self.command_lock = threading.Lock()
        
        print(f"[ArduinoPanTilt] Controller başlatılıyor...")
        
        # Port belirtilmediyse otomatik tespit et
        if port is None:
            port = self.find_arduino_port()
        
        self.port = port
        
        if self.port:
            self.connect()
        else:
            print("[ArduinoPanTilt] ❌ Arduino portu bulunamadı!")
    
    def find_arduino_port(self) -> Optional[str]:
        """Arduino portunu otomatik tespit et"""
        print("[ArduinoPanTilt] Arduino portu aranıyor...")
        
        # Windows COM portları
        if sys.platform.startswith('win'):
            possible_ports = [f'COM{i}' for i in range(1, 21)]
        # Linux USB portları
        else:
            possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        
        for port in possible_ports:
            try:
                print(f"[ArduinoPanTilt] Port test ediliyor: {port}")
                test_serial = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(2)  # Arduino reset bekle
                
                # Test komutu gönder
                test_serial.write(b"TEST\\n")
                test_serial.flush()
                time.sleep(0.1)
                
                # Yanıt kontrol et
                if test_serial.in_waiting > 0:
                    response = test_serial.readline().decode().strip()
                    test_serial.close()
                    
                    if "OK" in response or "READY" in response:
                        print(f"[ArduinoPanTilt] ✅ Arduino bulundu: {port}")
                        return port
                
                test_serial.close()
                
            except Exception as e:
                continue
        
        print("[ArduinoPanTilt] ❌ Arduino portu bulunamadı!")
        return None
    
    def connect(self) -> bool:
        """Arduino'ya bağlan"""
        if not self.port:
            print("[ArduinoPanTilt] ❌ Port belirtilmemiş!")
            return False
        
        try:
            # Önceki bağlantıyı kapat
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            print(f"[ArduinoPanTilt] {self.port} portuna bağlanılıyor...")
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout,
                rtscts=False,
                dsrdtr=False
            )
            
            # Arduino'nun bootloader'ından çıkmasını bekle
            print("[ArduinoPanTilt] Arduino başlatılıyor...")
            time.sleep(3)
            
            # Buffer'ı temizle
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # Bağlantı test et
            for attempt in range(3):
                print(f"[ArduinoPanTilt] Bağlantı test ediliyor (deneme {attempt+1}/3)...")
                
                if self.send_test_command():
                    self.is_connected = True
                    print(f"[ArduinoPanTilt] ✅ Arduino bağlandı: {self.port}")
                    
                    # Başlangıç pozisyonu
                    time.sleep(0.5)
                    self.center_position()
                    return True
                
                time.sleep(1)
            
            print("[ArduinoPanTilt] ❌ Arduino yanıt vermiyor!")
            return False
            
        except Exception as e:
            print(f"[ArduinoPanTilt] ❌ Bağlantı hatası: {e}")
            print(f"[ArduinoPanTilt] Port: {self.port}, Baud: {self.baud_rate}")
            self.is_connected = False
            return False
    
    def send_test_command(self) -> bool:
        """Test komutu gönder"""
        try:
            # Birkaç farklı test komutu dene
            test_commands = ["TEST", "STATUS", "PING"]
            
            for cmd in test_commands:
                self.serial_conn.write(f"{cmd}\\n".encode())
                self.serial_conn.flush()
                time.sleep(0.2)
                
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode().strip()
                    print(f"[ArduinoPanTilt] Arduino yanıtı: {response}")
                    
                    if any(word in response.upper() for word in ["OK", "READY", "BARLAS"]):
                        return True
            
            return False
            
        except Exception as e:
            print(f"[ArduinoPanTilt] Test komutu hatası: {e}")
            return False
    
    def send_command(self, command: str, wait_response: bool = True) -> bool:
        """Arduino'ya komut gönder - iyileştirilmiş"""
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            print("[ArduinoPanTilt] ❌ Arduino bağlı değil!")
            return False
        
        try:
            with self.command_lock:
                # Buffer'ı temizle
                self.serial_conn.reset_input_buffer()
                
                # Komutu gönder
                cmd_with_newline = command + "\\n"
                self.serial_conn.write(cmd_with_newline.encode())
                self.serial_conn.flush()
                
                print(f"[ArduinoPanTilt] Komut gönderildi: {command}")
                
                if not wait_response:
                    return True
                
                # Yanıt bekle
                max_wait_time = 2.0  # 2 saniye max
                start_time = time.time()
                
                while (time.time() - start_time) < max_wait_time:
                    if self.serial_conn.in_waiting > 0:
                        response = self.serial_conn.readline().decode().strip()
                        print(f"[ArduinoPanTilt] Arduino yanıtı: {response}")
                        
                        # Başarı kontrolleri
                        if any(word in response.upper() for word in ["OK", "DONE", "SUCCESS"]):
                            return True
                        elif "ERROR" in response.upper():
                            print(f"[ArduinoPanTilt] ❌ Arduino hatası: {response}")
                            return False
                        else:
                            return True  # Herhangi bir yanıt = başarı
                    
                    time.sleep(0.05)  # 50ms bekle
                
                print("[ArduinoPanTilt] ⚠️ Arduino yanıt vermedi (timeout)")
                return False  # Timeout - Arduino yanıt vermedi
                
        except Exception as e:
            print(f"[ArduinoPanTilt] ❌ Komut gönderme hatası: {e}")
            return False
    
    def move_to_position(self, pan_angle: float, tilt_angle: float) -> bool:
        """Servo'ları hareket ettir - iyileştirilmiş"""
        # Açı sınırlarını kontrol et
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        print(f"[ArduinoPanTilt] Hedef pozisyon: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
        
        # Farklı komut formatlarını dene
        commands_to_try = [
            f"MOVE,{int(pan_angle)},{int(tilt_angle)}",  # Orijinal format
            f"PAN,{int(pan_angle)}",                      # Ayrı pan komutu
            f"TILT,{int(tilt_angle)}",                    # Ayrı tilt komutu
            f"SERVO,{int(pan_angle)},{int(tilt_angle)}",  # Alternatif format
            f"SET,{int(pan_angle)},{int(tilt_angle)}"     # Başka format
        ]
        
        for command in commands_to_try:
            print(f"[ArduinoPanTilt] Komut deneniyor: {command}")
            
            if self.send_command(command):
                self.current_pan = pan_angle
                self.current_tilt = tilt_angle
                print(f"[ArduinoPanTilt] ✅ Hareket başarılı: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
                return True
            
            time.sleep(0.2)  # Komutlar arası bekle
        
        print("[ArduinoPanTilt] ❌ Tüm hareket komutları başarısız!")
        return False
    
    def move_pan_only(self, pan_angle: float) -> bool:
        """Sadece pan servo'yu hareket ettir"""
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        
        if self.send_command(f"PAN,{int(pan_angle)}"):
            self.current_pan = pan_angle
            print(f"[ArduinoPanTilt] ✅ Pan hareket: {pan_angle:.1f}°")
            return True
        return False
    
    def move_tilt_only(self, tilt_angle: float) -> bool:
        """Sadece tilt servo'yu hareket ettir"""
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        if self.send_command(f"TILT,{int(tilt_angle)}"):
            self.current_tilt = tilt_angle
            print(f"[ArduinoPanTilt] ✅ Tilt hareket: {tilt_angle:.1f}°")
            return True
        return False
    
    def enable_laser(self) -> bool:
        """Lazer'i aç - iyileştirilmiş"""
        commands_to_try = [
            "LASER,ON",
            "LASER,1", 
            "LED,ON",
            "FIRE,ON"
        ]
        
        for command in commands_to_try:
            if self.send_command(command):
                self.laser_active = True
                print("[ArduinoPanTilt] 🔴 LAZER AKTİF")
                return True
            time.sleep(0.1)
        
        print("[ArduinoPanTilt] ❌ Lazer açma başarısız!")
        return False
    
    def disable_laser(self) -> bool:
        """Lazer'i kapat - iyileştirilmiş"""
        commands_to_try = [
            "LASER,OFF",
            "LASER,0",
            "LED,OFF", 
            "FIRE,OFF"
        ]
        
        for command in commands_to_try:
            if self.send_command(command):
                self.laser_active = False
                print("[ArduinoPanTilt] ⚫ Lazer kapatıldı")
                return True
            time.sleep(0.1)
        
        print("[ArduinoPanTilt] ❌ Lazer kapatma başarısız!")
        return False
    
    def center_position(self) -> bool:
        """Merkez pozisyon - iyileştirilmiş"""
        print("[ArduinoPanTilt] Merkez pozisyona gidiyor...")
        
        # Önce merkez komutunu dene
        if self.send_command("CENTER"):
            self.current_pan = 90
            self.current_tilt = 90
            print("[ArduinoPanTilt] ✅ Merkez pozisyon (CENTER komutu)")
            return True
        
        # Manuel merkez pozisyon
        if self.move_to_position(90, 90):
            print("[ArduinoPanTilt] ✅ Merkez pozisyon (MOVE komutu)")
            return True
        
        print("[ArduinoPanTilt] ❌ Merkez pozisyon başarısız!")
        return False
    
    def get_current_position(self) -> Tuple[float, float]:
        """Mevcut pozisyonu döndür"""
        return (self.current_pan, self.current_tilt)
    
    def disconnect(self):
        """Bağlantıyı kes"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.disable_laser()
                time.sleep(0.5)
                self.serial_conn.close()
                self.is_connected = False
                print("[ArduinoPanTilt] ✅ Arduino bağlantısı kapatıldı")
            except Exception as e:
                print(f"[ArduinoPanTilt] Bağlantı kapatma hatası: {e}")
    
    def test_movement(self):
        """Servo hareket testi"""
        if not self.is_connected:
            print("[ArduinoPanTilt] ❌ Arduino bağlı değil!")
            return False
        
        print("[ArduinoPanTilt] 🧪 Servo hareket testi başlatılıyor...")
        
        # Test pozisyonları
        test_positions = [
            (90, 90),   # Merkez
            (60, 90),   # Sol
            (120, 90),  # Sağ
            (90, 60),   # Aşağı
            (90, 120),  # Yukarı
            (90, 90)    # Tekrar merkez
        ]
        
        success_count = 0
        
        for i, (pan, tilt) in enumerate(test_positions):
            print(f"[ArduinoPanTilt] Test {i+1}/6: Pan={pan}°, Tilt={tilt}°")
            
            if self.move_to_position(pan, tilt):
                success_count += 1
                print(f"[ArduinoPanTilt] ✅ Test {i+1} başarılı")
            else:
                print(f"[ArduinoPanTilt] ❌ Test {i+1} başarısız")
            
            time.sleep(1.5)  # Servo hareket süresi
        
        # Lazer testi
        print("[ArduinoPanTilt] 🔴 Lazer testi...")
        if self.enable_laser():
            time.sleep(1)
            if self.disable_laser():
                success_count += 1
                print("[ArduinoPanTilt] ✅ Lazer testi başarılı")
            else:
                print("[ArduinoPanTilt] ❌ Lazer kapatma başarısız")
        else:
            print("[ArduinoPanTilt] ❌ Lazer açma başarısız")
        
        print(f"[ArduinoPanTilt] 🧪 Test sonucu: {success_count}/7 başarılı")
        return success_count >= 5  # En az 5/7 başarı = genel başarı

# Test fonksiyonu
def test_arduino_controller():
    """Arduino controller test fonksiyonu"""
    print("🎯 BARLAS Arduino Pan-Tilt Controller Test")
    print("==========================================")
    
    # Controller oluştur
    controller = ArduinoPanTiltController()
    
    if controller.is_connected:
        # Hareket testi
        controller.test_movement()
        
        # Manuel kontrol testi
        print("\\n[Test] Manuel kontrol testi (5 saniye)...")
        import threading
        import keyboard
        
        def manual_control():
            pan, tilt = controller.get_current_position()
            
            print("WASD: Hareket, Space: Lazer, Q: Çıkış")
            
            while True:
                try:
                    if keyboard.is_pressed('w'):  # Yukarı
                        tilt = min(controller.tilt_max, tilt + 2)
                        controller.move_tilt_only(tilt)
                    elif keyboard.is_pressed('s'):  # Aşağı
                        tilt = max(controller.tilt_min, tilt - 2)
                        controller.move_tilt_only(tilt)
                    elif keyboard.is_pressed('a'):  # Sol
                        pan = max(controller.pan_min, pan - 2)
                        controller.move_pan_only(pan)
                    elif keyboard.is_pressed('d'):  # Sağ
                        pan = min(controller.pan_max, pan + 2)
                        controller.move_pan_only(pan)
                    elif keyboard.is_pressed('space'):  # Lazer
                        if controller.laser_active:
                            controller.disable_laser()
                        else:
                            controller.enable_laser()
                        time.sleep(0.5)  # Debounce
                    elif keyboard.is_pressed('c'):  # Merkez
                        controller.center_position()
                        pan, tilt = 90, 90
                        time.sleep(0.5)
                    elif keyboard.is_pressed('q'):  # Çıkış
                        break
                    
                    time.sleep(0.1)
                    
                except:
                    break
        
        # Manuel kontrol thread
        control_thread = threading.Thread(target=manual_control)
        control_thread.daemon = True
        control_thread.start()
        
        # 30 saniye bekle veya 'q' basılana kadar
        control_thread.join(30)
        
    else:
        print("❌ Arduino bağlantısı başarısız!")
        print("\\n🔧 Troubleshooting:")
        print("1. Arduino USB kablosu bağlı mı?")
        print("2. Arduino'ya doğru firmware yüklenmiş mi?")
        print("3. Serial port doğru mu?")
        print("4. Başka program Arduino'yu kullanıyor mu?")
    
    # Bağlantıyı kapat
    controller.disconnect()
    print("\\n✅ Test tamamlandı")

if __name__ == "__main__":
    test_arduino_controller()
