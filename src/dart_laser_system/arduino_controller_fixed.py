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
            return True
        except Exception as e:
            print(f"[ArduinoController] ❌ Bağlantı hatası: {e}")
            return False

    def disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def send_command(self, cmd: str):
        if self.ser:
            self.ser.write((cmd + "\n").encode())
            return True
        return False

    def move_to_position(self, pan, tilt):
        self.current_pan = int(pan)
        self.current_tilt = int(tilt)
        return self.send_command(f"MOVE,{self.current_pan},{self.current_tilt}")

    def center_position(self):
        return self.move_to_position(90, 90)

    def enable_laser(self):
        self.laser_active = True
        return self.send_command("LASER,ON")

    def disable_laser(self):
        self.laser_active = False
        return self.send_command("LASER,OFF")
