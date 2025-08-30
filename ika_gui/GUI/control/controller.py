import serial
import time

class VehicleController:
    def __init__(self, port="COM3", baudrate=9600):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            print(f"[OK] {port} üzerinden bağlantı kuruldu.")
            self.connected = True
        except serial.SerialException as e:
            print(f"[HATA] Bağlantı kurulamadı: {e}")
            self.ser = None
            self.connected = False

        # Sensör verileri
        self.lat, self.lon = 0.0, 0.0
        self.speed = 0.0
        self.heading = 0.0
        self.imu = (0.0, 0.0, 0.0)

    # --- Manuel komutlar ---
    def send_command(self, command):
        if self.ser and self.connected:
            self.ser.write((command + "\n").encode())
            print(f"[TX] {command}")
        else:
            print("[!] Seri bağlantı yok, komut gönderilemedi.")

    def forward(self): self.send_command("FORWARD")
    def backward(self): self.send_command("BACKWARD")
    def left(self): self.send_command("LEFT")
    def right(self): self.send_command("RIGHT")
    def stop(self): self.send_command("STOP")

    # --- Sensör verisi okuma ---
    def read_serial(self):
        if self.ser and self.connected:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    # Örn: LAT:39.9334,LON:32.8597,SPEED:1.2,HEADING:90,IMU_X:0.01,IMU_Y:-0.02,IMU_Z:9.81
                    parts = {kv.split(":")[0]: kv.split(":")[1] for kv in line.split(",")}
                    self.lat = float(parts.get("LAT", self.lat))
                    self.lon = float(parts.get("LON", self.lon))
                    self.speed = float(parts.get("SPEED", self.speed))
                    self.heading = float(parts.get("HEADING", self.heading))
                    self.imu = (
                        float(parts.get("IMU_X", self.imu[0])),
                        float(parts.get("IMU_Y", self.imu[1])),
                        float(parts.get("IMU_Z", self.imu[2])),
                    )
            except Exception as e:
                print(f"[!] Veri parse edilemedi: {e}")

    # --- Getter metodları ---
    def get_gps(self): return self.lat, self.lon
    def get_speed(self): return self.speed
    def get_heading(self): return self.heading
    def get_imu(self): return self.imu

    # --- Bağlantı kontrol ---
    def is_connected(self): 
        return self.connected and self.ser is not None
