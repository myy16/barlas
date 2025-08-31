# control/controller.py
import time, threading
import serial

# optional libs
try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except Exception:
    PYMAVLINK_AVAILABLE = False

try:
    import requests
    REQUESTS_AVAILABLE = True
except Exception:
    REQUESTS_AVAILABLE = False

class VehicleController:
    """
    Unified controller:
      - Seri (Arduino) üzerinden encoder okur ve manuel komut gönderir.
      - Eğer pixhawk_url verildi ve pymavlink mevcutsa, Pixhawk'tan konum/hız/heading okur.
      - Eğer rpi_url verildi ve requests mevcutsa, RPi üzerindeki /sensors endpoint'inden veri çeker.
    """

    def __init__(self, port="COM3", baudrate=9600, pixhawk_url=None, rpi_url=None, use_pixhawk=False, use_rpi=False):
        # Seri (Arduino) bağlantısı
        self.ser = None
        self.connected = False
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(1.5)
            self.connected = True
            print(f"[OK] Seri bağlı: {port}")
        except Exception as e:
            print(f"[WARN] Seri bağlanamadı ({port}): {e}")
            self.ser = None
            self.connected = False

        # durum / sensör state
        self._lock = threading.Lock()
        self.lat, self.lon = 0.0, 0.0
        self.speed = 0.0      # km/h
        self.heading = 0.0    # deg
        self.imu = (0.0, 0.0, 0.0)
        self.lidar_min = None
        self.ultrasonic_cm = None
        self.enc_l = None
        self.enc_r = None

        # pixhawk
        self.pixhawk_url = pixhawk_url
        self.use_pixhawk = use_pixhawk and PYMAVLINK_AVAILABLE and (pixhawk_url is not None)
        self.pixhawk_master = None
        self.pixhawk_connected = False

        # rpi HTTP server
        self.rpi_url = rpi_url
        self.use_rpi = use_rpi and REQUESTS_AVAILABLE and (rpi_url is not None)
        self.rpi_connected = False

        # başlat
        if self.ser:
            t = threading.Thread(target=self._serial_reader_loop, daemon=True)
            t.start()

        if self.use_pixhawk:
            t2 = threading.Thread(target=self._pixhawk_loop, daemon=True)
            t2.start()
        else:
            if pixhawk_url and not PYMAVLINK_AVAILABLE:
                print("[INFO] pymavlink yüklü değil; Pixhawk desteği devre dışı.")

        if self.use_rpi:
            t3 = threading.Thread(target=self._rpi_poll_loop, daemon=True)
            t3.start()
        else:
            if rpi_url and not REQUESTS_AVAILABLE:
                print("[INFO] requests yüklü değil; RPi HTTP desteği devre dışı.")

    # --- SERIAL: Arduino'dan ENCL:...,ENCR:... satırlarını oku ---
    def _serial_reader_loop(self):
        while True:
            try:
                if not self.ser:
                    time.sleep(0.2)
                    continue
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    time.sleep(0.01)
                    continue
                # Örnek: ENCL:123,ENCR:125 or LAT:...,LON:...,IMU_X:...,SPEED:...
                parts = {}
                for kv in line.split(","):
                    if ":" in kv:
                        k, v = kv.split(":", 1)
                        parts[k.strip().upper()] = v.strip()

                with self._lock:
                    if "ENCL" in parts:
                        try: self.enc_l = int(parts["ENCL"])
                        except: self.enc_l = None
                    if "ENCR" in parts:
                        try: self.enc_r = int(parts["ENCR"])
                        except: self.enc_r = None

                    # Arduino doğrudan GPS/IMU vs. de yollayabiliyorsa yakala
                    if "LAT" in parts and "LON" in parts:
                        try:
                            self.lat = float(parts["LAT"]); self.lon = float(parts["LON"])
                        except: pass
                    if "SPEED" in parts:
                        try: self.speed = float(parts["SPEED"])
                        except: pass
                    if "HEADING" in parts:
                        try: self.heading = float(parts["HEADING"])
                        except: pass
                    if "IMU_X" in parts and "IMU_Y" in parts and "IMU_Z" in parts:
                        try:
                            self.imu = (float(parts["IMU_X"]), float(parts["IMU_Y"]), float(parts["IMU_Z"]))
                        except: pass
            except Exception as e:
                print(f"[SERIAL] Hata: {e}")
                time.sleep(0.2)

    # --- PIXHAWK via pymavlink (opsiyonel) ---
    def _pixhawk_loop(self):
        try:
            self.pixhawk_master = mavutil.mavlink_connection(self.pixhawk_url)
            self.pixhawk_master.wait_heartbeat(timeout=10)
            self.pixhawk_connected = True
            print("[OK] Pixhawk bağlandı:", self.pixhawk_url)
        except Exception as e:
            print("[PIXHAWK] Bağlanamadı:", e)
            self.pixhawk_connected = False
            return

        while True:
            try:
                msg = self.pixhawk_master.recv_match(timeout=1)
                if not msg:
                    continue
                mtype = msg.get_type()
                with self._lock:
                    if mtype == "GLOBAL_POSITION_INT":
                        # lat/lon in 1e7
                        try:
                            self.lat = msg.lat / 1e7
                            self.lon = msg.lon / 1e7
                        except: pass
                    elif mtype == "VFR_HUD":
                        try:
                            self.speed = float(msg.groundspeed) * 3.6
                            self.heading = float(msg.heading)
                        except: pass
                    elif mtype == "HEARTBEAT":
                        pass
            except Exception as e:
                print("[PIXHAWK] Loop hata:", e)
                time.sleep(0.5)

    # --- RPi HTTP polling (Flask server'dan /sensors JSON bekler) ---
    def _rpi_poll_loop(self):
        url = self.rpi_url.rstrip("/") + "/sensors"
        while True:
            try:
                r = requests.get(url, timeout=1.0)
                if r.status_code == 200:
                    data = r.json()
                    with self._lock:
                        if "lidar_min" in data: self.lidar_min = float(data["lidar_min"])
                        if "ultrasonic_cm" in data: self.ultrasonic_cm = float(data["ultrasonic_cm"])
                        if "imu" in data and isinstance(data["imu"], dict):
                            imud = data["imu"]
                            self.imu = (float(imud.get("x", self.imu[0])), float(imud.get("y", self.imu[1])), float(imud.get("z", self.imu[2])))
                        # RPi ayrıca GPS ya da diğerleri yollayabilir - handle if exists
                        if "lat" in data and "lon" in data:
                            try: self.lat = float(data["lat"]); self.lon = float(data["lon"])
                            except: pass
                    self.rpi_connected = True
                else:
                    self.rpi_connected = False
            except Exception:
                self.rpi_connected = False
            time.sleep(0.3)

    # --- Manuel komutlar: seri üzerinden gönder ---
    def send_command(self, command):
        if self.ser and self.connected:
            try:
                self.ser.write((command + "\n").encode())
                # optional: wait ack from arduino
                return True
            except Exception as e:
                print("[TX ERR]", e)
                return False
        else:
            print("[TX] Seri bağlı değil.")
            return False

    def forward(self):  return self.send_command("FORWARD")
    def backward(self): return self.send_command("BACKWARD")
    def left(self):     return self.send_command("LEFT")
    def right(self):    return self.send_command("RIGHT")
    def stop(self):     return self.send_command("STOP")

    # --- Getterlar (thread-safe) ---
    def get_gps(self):
        with self._lock: return self.lat, self.lon
    def get_speed(self):
        with self._lock: return self.speed
    def get_heading(self):
        with self._lock: return self.heading
    def get_imu(self):
        with self._lock: return self.imu
    def get_lidar_min(self):
        with self._lock: return self.lidar_min
    def get_ultrasonic_cm(self):
        with self._lock: return self.ultrasonic_cm
    def get_encoders(self):
        with self._lock: return self.enc_l, self.enc_r

    # --- durumlar ---
    def is_connected(self): return bool(self.ser) and self.connected
    def is_pixhawk_connected(self): return self.pixhawk_connected
    def is_rpi_connected(self): return self.rpi_connected
