import serial
import time

class ArduinoPanTiltController:
    def __init__(self, port="/dev/ttyACM0", baud_rate=9600, timeout=2):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.current_pan = 90
        self.current_tilt = 90
        self.laser_active = False

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            time.sleep(2)  # Arduino reset bekle
            
            # Bağlantı testi
            if self.test_connection():
                print(f"[ArduinoController] ✅ Bağlandı: {self.port}")
                return True
            else:
                print(f"[ArduinoController] ❌ Arduino yanıt vermiyor: {self.port}")
                self.ser.close()
                return False
                
        except Exception as e:
            print(f"[ArduinoController] ❌ Bağlantı hatası: {e}")
            return False
    
    def test_connection(self):
        """Arduino bağlantısını test et"""
        try:
            # Buffer temizle
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Test komutu gönder
            self.ser.write(b"TEST\n")
            self.ser.flush()
            time.sleep(0.5)
            
            # Yanıt kontrol et
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                print(f"[ArduinoController] Arduino yanıtı: '{response}'")
                return "OK" in response or "Ready" in response
            else:
                print("[ArduinoController] ❌ Arduino yanıt vermiyor")
                return False
        except Exception as e:
            print(f"[ArduinoController] ❌ Test hatası: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None

    def send_command(self, cmd: str):
        """Arduino'ya komut gönder ve yanıt al"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode())
                self.ser.flush()
                time.sleep(0.1)  # Arduino işlem süresi
                
                # Yanıt oku
                if self.ser.in_waiting > 0:
                    response = self.ser.readline().decode().strip()
                    print(f"[Arduino] → {cmd} ← {response}")
                    return response
                return "OK"
            except Exception as e:
                print(f"[Arduino] ❌ Komut hatası: {e}")
                return None
        else:
            print("[Arduino] ❌ Bağlantı yok!")
            return None

    def move_to_position(self, pan, tilt):
        """Servo'ları belirtilen açılara hareket ettir"""
        self.current_pan = int(pan)
        self.current_tilt = int(tilt)
        response = self.send_command(f"MOVE,{self.current_pan},{self.current_tilt}")
        return response == "Moved"

    def center_position(self):
        """Servo'ları merkez pozisyona getir"""
        return self.move_to_position(90, 90)

    def enable_laser(self):
        """Lazeri aç"""
        self.laser_active = True
        response = self.send_command("LASER,ON")
        return response == "Laser ON"

    def disable_laser(self):
        """Lazeri kapat"""
        self.laser_active = False
        response = self.send_command("LASER,OFF")
        return response == "Laser OFF"
