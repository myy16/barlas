"""
KTR (Kontrol Transfer Robotu) ile seri haberleşme modülü
Sensör verilerinin alınması ve komutların gönderilmesi
"""
import time
import serial
import threading
import json
from typing import Dict, Optional, List, Any
from dataclasses import dataclass

@dataclass
class SensorData:
    """Sensör verisi sınıfı"""
    timestamp: float
    ultrasonic: List[float]  # 8 adet ultrasonik sensör mesafesi
    imu: Dict[str, float]    # IMU verileri (roll, pitch, yaw, acc_x, acc_y, acc_z)
    temperature: float       # Sıcaklık (°C)
    battery: Dict[str, float]  # Batarya durumu (voltage, current, percentage)
    lidar_scan: Optional[List[float]] = None  # LIDAR tarama verileri (opsiyonel)

class KTRInterface:
    """KTR ile seri haberleşme sınıfı"""
    
    # Komut protokolü
    CMD_START = b'<'
    CMD_END = b'>'
    
    # Komut tipleri
    CMD_GET_SENSORS = b'GET_SENSORS'
    CMD_SET_MOTOR = b'SET_MOTOR'
    CMD_EMERGENCY_STOP = b'STOP'
    
    def __init__(self, port: str = "COM3", baudrate: int = 115200):
        """
        Args:
            port (str): Seri port adı
            baudrate (int): Haberleşme hızı
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.read_thread = None
        self.lock = threading.Lock()
        
        # Son sensör verisi
        self.last_sensor_data = None
        self.last_read_time = 0
        
        # Hata sayacı
        self.error_count = 0
        self.max_errors = 10
        
    def connect(self) -> bool:
        """KTR'ye bağlan
        
        Returns:
            bool: Bağlantı başarılı mı
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            
            # Test komutu gönder
            if self._send_command(self.CMD_GET_SENSORS):
                print(f"✅ KTR bağlantısı başarılı: {self.port}")
                return True
            else:
                print("❌ KTR yanıt vermiyor!")
                return False
                
        except Exception as e:
            print(f"❌ KTR bağlantı hatası: {e}")
            return False
    
    def start_reading(self):
        """Sürekli okuma döngüsünü başlat"""
        if not self.running and self.serial:
            self.running = True
            self.read_thread = threading.Thread(target=self._reading_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            print("✅ KTR veri okuma başlatıldı")
    
    def stop_reading(self):
        """Okuma döngüsünü durdur"""
        self.running = False
        if self.read_thread:
            self.read_thread.join()
        print("⏹️ KTR veri okuma durduruldu")
    
    def _reading_loop(self):
        """Sürekli veri okuyan döngü"""
        while self.running:
            try:
                # Sensör verisi iste
                if self._send_command(self.CMD_GET_SENSORS):
                    # Yanıt bekle
                    response = self._read_response()
                    if response:
                        # JSON verisini ayrıştır
                        data = json.loads(response)
                        
                        # SensorData nesnesine dönüştür
                        sensor_data = SensorData(
                            timestamp=time.time(),
                            ultrasonic=data['ultrasonic'],
                            imu=data['imu'],
                            temperature=data['temperature'],
                            battery=data['battery'],
                            lidar_scan=data.get('lidar_scan')
                        )
                        
                        # Veriyi güncelle
                        with self.lock:
                            self.last_sensor_data = sensor_data
                            self.last_read_time = sensor_data.timestamp
                            self.error_count = 0  # Hata sayacını sıfırla
                            
                else:
                    self._handle_error("Komut gönderilemedi")
                    
            except Exception as e:
                self._handle_error(f"Veri okuma hatası: {e}")
                
            # 100ms bekle (10Hz güncelleme)
            time.sleep(0.1)
    
    def _send_command(self, cmd: bytes, data: Dict[str, Any] = None) -> bool:
        """KTR'ye komut gönder
        
        Args:
            cmd (bytes): Komut tipi
            data (dict, optional): Komut parametreleri
            
        Returns:
            bool: Gönderim başarılı mı
        """
        try:
            # Komutu hazırla
            if data:
                command = self.CMD_START + cmd + b':' + json.dumps(data).encode() + self.CMD_END
            else:
                command = self.CMD_START + cmd + self.CMD_END
            
            # Gönder
            self.serial.write(command)
            self.serial.flush()
            return True
            
        except Exception as e:
            print(f"⚠️ Komut gönderme hatası: {e}")
            return False
    
    def _read_response(self) -> Optional[str]:
        """KTR'den yanıt oku
        
        Returns:
            str: JSON formatında yanıt verisi
        """
        try:
            # Başlangıç karakterini bekle
            while self.serial.read() != self.CMD_START:
                pass
            
            # Bitiş karakterine kadar oku
            response = b''
            while True:
                char = self.serial.read()
                if char == self.CMD_END:
                    break
                response += char
            
            return response.decode()
            
        except Exception as e:
            print(f"⚠️ Yanıt okuma hatası: {e}")
            return None
    
    def _handle_error(self, error_msg: str):
        """Hata durumunu yönet"""
        print(f"⚠️ {error_msg}")
        self.error_count += 1
        
        # Çok fazla hata varsa bağlantıyı yenile
        if self.error_count >= self.max_errors:
            print("🔄 Çok fazla hata! Bağlantı yenileniyor...")
            self.reconnect()
    
    def reconnect(self):
        """Bağlantıyı yeniden kur"""
        try:
            if self.serial:
                self.serial.close()
            time.sleep(1.0)
            self.connect()
            self.error_count = 0
        except:
            pass
    
    def set_motor_speeds(self, left: float, right: float) -> bool:
        """Motor hızlarını ayarla
        
        Args:
            left (float): Sol motor hızı (-1.0 ile 1.0 arası)
            right (float): Sağ motor hızı (-1.0 ile 1.0 arası)
            
        Returns:
            bool: Komut başarılı mı
        """
        # Hızları sınırla
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        
        # Komutu gönder
        return self._send_command(self.CMD_SET_MOTOR, {
            'left': left,
            'right': right
        })
    
    def emergency_stop(self) -> bool:
        """Acil duruş komutu gönder
        
        Returns:
            bool: Komut başarılı mı
        """
        return self._send_command(self.CMD_EMERGENCY_STOP)
    
    def get_last_sensor_data(self) -> Optional[SensorData]:
        """Son sensör verisini döndür"""
        with self.lock:
            return self.last_sensor_data
    
    def get_reading_age(self) -> float:
        """Son okumadan bu yana geçen süreyi döndür (saniye)"""
        with self.lock:
            return time.time() - self.last_read_time
    
    def cleanup(self):
        """Kaynakları temizle"""
        self.stop_reading()
        if self.serial:
            self.serial.close()
