"""
BARLAS Arduino Pan-Tilt Controller
Arduino Uno ile servo motor kontrolü
Serial kommunikasyon üzerinden komut gönderimi
"""
import serial
import time
import threading
from typing import Optional, Tuple
import json

class ArduinoPanTiltController:
    """
    Arduino Uno ile Pan-Tilt servo kontrolü
    Serial port üzerinden komut gönderir
    """
    
    def __init__(self, port='COM3', baud_rate=9600, timeout=2):
        """
        Arduino Controller
        
        Args:
            port: Arduino serial port (Windows: COM3, Linux: /dev/ttyUSB0)
            baud_rate: Serial baud rate (varsayılan: 9600)
            timeout: Serial timeout (saniye)
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        
        # Serial bağlantı
        self.serial_conn = None
        self.is_connected = False
        
        # Servo pozisyonları
        self.pan_position = 90   # Başlangıç merkez
        self.tilt_position = 90  # Başlangıç merkez
        
        # Servo limitleri
        self.pan_min = 10
        self.pan_max = 170
        self.tilt_min = 30
        self.tilt_max = 150
        
        # Lazer durumu
        self.laser_active = False
        
        # Kalibrasyon değerleri
        self.calibration_offset_x = 0
        self.calibration_offset_y = 0
        
        # Thread lock
        self.command_lock = threading.Lock()
        
        print(f"[ArduinoController] Arduino Pan-Tilt Controller başlatılıyor...")
        self.connect()
    
    def connect(self):
        """Arduino'ya bağlan"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            # Arduino'nun başlamasını bekle
            time.sleep(2)
            
            # Test komutu gönder
            if self.send_command("TEST"):
                self.is_connected = True
                print(f"[ArduinoController] ✅ Arduino bağlandı: {self.port}")
                
                # Başlangıç pozisyonu
                self.move_to_position(self.pan_position, self.tilt_position)
                return True
            else:
                print(f"[ArduinoController] ❌ Arduino yanıt vermiyor")
                return False
                
        except Exception as e:
            print(f"[ArduinoController] ❌ Bağlantı hatası: {e}")
            print(f"[ArduinoController] Port kontrol edin: {self.port}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Arduino bağlantısını kes"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.disable_laser()
                self.serial_conn.close()
                self.is_connected = False
                print("[ArduinoController] Arduino bağlantısı kapatıldı")
            except Exception as e:
                print(f"[ArduinoController] Bağlantı kapatma hatası: {e}")
    
    def send_command(self, command: str) -> bool:
        """
        Arduino'ya komut gönder
        
        Args:
            command: Gönderilecek komut string
            
        Returns:
            bool: Başarı durumu
        """
        if not self.is_connected or not self.serial_conn:
            print("[ArduinoController] ❌ Arduino bağlı değil!")
            return False
        
        try:
            with self.command_lock:
                # Komutu gönder
                self.serial_conn.write((command + '\n').encode())
                self.serial_conn.flush()
                
                # Yanıt bekle
                response = self.serial_conn.readline().decode().strip()
                
                if response == "OK" or response.startswith("OK"):
                    return True
                else:
                    print(f"[ArduinoController] Arduino yanıtı: {response}")
                    return response != ""
                    
        except Exception as e:
            print(f"[ArduinoController] Komut gönderme hatası: {e}")
            return False
    
    def move_to_position(self, pan_angle: float, tilt_angle: float) -> bool:
        """
        Servo'ları belirtilen açılara hareket ettir
        
        Args:
            pan_angle: Pan servo açısı (0-180)
            tilt_angle: Tilt servo açısı (0-180)
            
        Returns:
            bool: Başarı durumu
        """
        # Açı sınırlarını kontrol et
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        # Komut formatı: "MOVE,pan_angle,tilt_angle"
        command = f"MOVE,{int(pan_angle)},{int(tilt_angle)}"
        
        if self.send_command(command):
            self.pan_position = pan_angle
            self.tilt_position = tilt_angle
            print(f"[ArduinoController] Pozisyon: Pan:{pan_angle:.1f}°, Tilt:{tilt_angle:.1f}°")
            return True
        else:
            print(f"[ArduinoController] ❌ Hareket komutu başarısız!")
            return False
    
    def enable_laser(self) -> bool:
        """Lazer'i aç"""
        if self.send_command("LASER,ON"):
            self.laser_active = True
            print("[ArduinoController] 🔴 LAZER AKTİF")
            return True
        else:
            print("[ArduinoController] ❌ Lazer açma başarısız!")
            return False
    
    def disable_laser(self) -> bool:
        """Lazer'i kapat"""
        if self.send_command("LASER,OFF"):
            self.laser_active = False
            print("[ArduinoController] ⚫ Lazer kapatıldı")
            return True
        else:
            print("[ArduinoController] ❌ Lazer kapatma başarısız!")
            return False
    
    def center_position(self) -> bool:
        """Servo'ları merkez pozisyona getir"""
        return self.move_to_position(90, 90)
    
    def pixel_to_angle(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> Tuple[float, float]:
        """
        Piksel koordinatlarını servo açılarına çevir
        
        Args:
            pixel_x, pixel_y: Hedef piksel koordinatları
            frame_width, frame_height: Kamera çözünürlüğü
            
        Returns:
            Tuple[float, float]: (pan_angle, tilt_angle)
        """
        # Merkez noktasından fark
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # Kalibrasyon offset'i uygula
        offset_x = pixel_x - center_x + self.calibration_offset_x
        offset_y = pixel_y - center_y + self.calibration_offset_y
        
        # Piksel farkını açıya çevir
        # Kamera FOV: 60° yatay, 45° dikey
        horizontal_fov = 60
        vertical_fov = 45
        
        pan_adjustment = (offset_x / center_x) * (horizontal_fov / 2)
        tilt_adjustment = -(offset_y / center_y) * (vertical_fov / 2)  # Y ters
        
        # Mevcut pozisyona ekle
        target_pan = self.pan_position + pan_adjustment
        target_tilt = self.tilt_position + tilt_adjustment
        
        return target_pan, target_tilt
    
    def aim_at_pixel(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> bool:
        """
        Belirtilen piksel koordinatına nişan al
        
        Args:
            pixel_x, pixel_y: Hedef piksel koordinatları
            frame_width, frame_height: Kamera çözünürlüğü
            
        Returns:
            bool: Başarı durumu
        """
        target_pan, target_tilt = self.pixel_to_angle(pixel_x, pixel_y, frame_width, frame_height)
        
        print(f"[ArduinoController] Hedef piksel: ({pixel_x}, {pixel_y})")
        print(f"[ArduinoController] Servo hedefi: Pan:{target_pan:.1f}°, Tilt:{target_tilt:.1f}°")
        
        if self.move_to_position(target_pan, target_tilt):
            # Lazer'i aktif et
            return self.enable_laser()
        return False
    
    def calibrate_offset(self, pixel_offset_x: float, pixel_offset_y: float):
        """Kamera-lazer offset kalibrasyonu"""
        self.calibration_offset_x = pixel_offset_x
        self.calibration_offset_y = pixel_offset_y
        print(f"[ArduinoController] Kalibrasyon: X:{pixel_offset_x}, Y:{pixel_offset_y}")
    
    def get_status(self) -> dict:
        """Sistem durumunu döndür"""
        return {
            'connected': self.is_connected,
            'port': self.port,
            'pan_position': self.pan_position,
            'tilt_position': self.tilt_position,
            'laser_active': self.laser_active,
            'calibration_x': self.calibration_offset_x,
            'calibration_y': self.calibration_offset_y
        }
    
    def manual_control(self):
        """Manuel kontrol modu"""
        print("\n🎮 MANUEL KONTROL MODU")
        print("=" * 40)
        print("Komutlar:")
        print("  w/s - Tilt up/down")
        print("  a/d - Pan left/right") 
        print("  space - Lazer aç/kapat")
        print("  c - Merkez pozisyon")
        print("  q - Çıkış")
        print("=" * 40)
        
        try:
            import msvcrt  # Windows için
            
            while True:
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8').lower()
                    
                    if key == 'q':
                        break
                    elif key == 'w':
                        self.move_to_position(self.pan_position, self.tilt_position - 5)
                    elif key == 's':
                        self.move_to_position(self.pan_position, self.tilt_position + 5)
                    elif key == 'a':
                        self.move_to_position(self.pan_position - 5, self.tilt_position)
                    elif key == 'd':
                        self.move_to_position(self.pan_position + 5, self.tilt_position)
                    elif key == ' ':
                        if self.laser_active:
                            self.disable_laser()
                        else:
                            self.enable_laser()
                    elif key == 'c':
                        self.center_position()
                
                time.sleep(0.1)
                
        except ImportError:
            print("Manuel kontrol sadece Windows'ta desteklenir")
        except KeyboardInterrupt:
            print("\nManuel kontrolden çıkılıyor...")
    
    def cleanup(self):
        """Temizlik"""
        self.disable_laser()
        self.center_position()
        time.sleep(1)
        self.disconnect()


def test_arduino_controller():
    """Arduino controller test fonksiyonu"""
    print("=" * 50)
    print("🔴 ARDUINO PAN-TILT CONTROLLER TEST")
    print("=" * 50)
    
    # Mevcut COM portlarını listele (Windows için)
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        print("Mevcut COM portları:")
        for port, desc, hwid in sorted(ports):
            print(f"  {port}: {desc}")
        print()
    except:
        pass
    
    # Port seçimi
    port = input("Arduino port'u (varsayılan COM3): ").strip() or "COM3"
    
    try:
        # Controller oluştur
        arduino = ArduinoPanTiltController(port=port)
        
        if not arduino.is_connected:
            print("❌ Arduino bağlanamadı! Port ve kablo kontrolü yapın.")
            return
        
        print("\n🎯 Test senaryoları:")
        
        # 1. Merkez pozisyon
        print("\n1. Merkez pozisyon")
        arduino.center_position()
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
            arduino.move_to_position(pan, tilt)
            time.sleep(2)
        
        # 3. Lazer testi
        print(f"\n3. Lazer testi")
        arduino.enable_laser()
        time.sleep(3)
        arduino.disable_laser()
        
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
            arduino.aim_at_pixel(px, py, 640, 480)
            time.sleep(2)
            arduino.disable_laser()
            time.sleep(1)
        
        # 5. Manuel kontrol testi
        print(f"\n5. Manuel kontrol test etmek ister misiniz? (y/n)")
        if input().lower().startswith('y'):
            arduino.manual_control()
        
        print(f"\n✅ Test tamamlandı!")
        
        # Durum bilgisi
        status = arduino.get_status()
        print(f"\n📊 Arduino Durumu:")
        for key, value in status.items():
            print(f"  {key}: {value}")
        
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if 'arduino' in locals():
            arduino.cleanup()


if __name__ == "__main__":
    test_arduino_controller()
