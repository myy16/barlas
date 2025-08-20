"""
INA219 Batarya Sensörü Yönetim Modülü
Akım, voltaj ve güç ölçümü
"""
import time
import threading
from typing import Dict, Optional
import board
import busio
from adafruit_ina219 import INA219

class BatterySensor:
    """INA219 batarya sensörü yönetim sınıfı"""
    
    def __init__(self, i2c_addr: int = 0x40):
        """
        Args:
            i2c_addr (int): I2C adres (varsayılan: 0x40)
        """
        self.i2c_addr = i2c_addr
        self.i2c = None
        self.sensor = None
        
        # Okuma parametreleri
        self.reading = False
        self.read_thread = None
        self.lock = threading.Lock()
        
        # Son ölçüm değerleri
        self.last_reading = None
        self.last_read_time = 0
        
        # Çalışma parametreleri
        self.read_interval = 0.5  # saniye
        
    def initialize(self) -> bool:
        """Sensörü başlat ve hazırla
        
        Returns:
            bool: Başlatma başarılı mı
        """
        try:
            # I2C arayüzünü başlat
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # INA219 sensörünü yapılandır
            self.sensor = INA219(self.i2c, addr=self.i2c_addr)
            
            # Ölçüm aralıklarını ayarla
            self.sensor.bus_voltage_range = INA219.RANGE_32V
            self.sensor.gain = INA219.GAIN_AUTO
            self.sensor.bus_adc_resolution = INA219.ADC_128SAMPS
            self.sensor.shunt_adc_resolution = INA219.ADC_128SAMPS
            
            print(f"✅ INA219 sensörü başlatıldı (0x{self.i2c_addr:02X})")
            return True
            
        except Exception as e:
            print(f"❌ Sensör başlatma hatası: {e}")
            return False
    
    def read_measurements(self) -> Optional[Dict[str, float]]:
        """Tüm ölçümleri yap ve döndür
        
        Returns:
            Dict with:
            - 'bus_voltage': V
            - 'shunt_voltage': mV
            - 'current': mA
            - 'power': mW
        """
        if not self.sensor:
            return None
            
        try:
            measurements = {
                'bus_voltage': self.sensor.bus_voltage,      # V
                'shunt_voltage': self.sensor.shunt_voltage,  # mV
                'current': self.sensor.current,              # mA
                'power': self.sensor.power                   # mW
            }
            return measurements
            
        except Exception as e:
            print(f"⚠️ Ölçüm hatası: {e}")
            return None
    
    def start_reading(self):
        """Sürekli okuma döngüsünü başlat"""
        if not self.reading:
            self.reading = True
            self.read_thread = threading.Thread(target=self._reading_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            print("✅ Batarya ölçümleri başlatıldı")
    
    def stop_reading(self):
        """Okuma döngüsünü durdur"""
        self.reading = False
        if self.read_thread:
            self.read_thread.join()
        print("⏹️ Batarya ölçümleri durduruldu")
    
    def _reading_loop(self):
        """Sürekli ölçüm yapan döngü"""
        while self.reading:
            measurements = self.read_measurements()
            
            with self.lock:
                self.last_reading = measurements
                self.last_read_time = time.time()
            
            time.sleep(self.read_interval)
    
    def get_last_reading(self) -> Optional[Dict[str, float]]:
        """Son ölçüm değerlerini döndür"""
        with self.lock:
            return self.last_reading
    
    def get_reading_age(self) -> float:
        """Son ölçümden bu yana geçen süreyi döndür (saniye)"""
        with self.lock:
            return time.time() - self.last_read_time
    
    def get_battery_percentage(self) -> Optional[float]:
        """Batarya yüzdesini hesapla (12V batarya için)
        
        Returns:
            float: 0-100 arası yüzde değeri
        """
        reading = self.get_last_reading()
        if not reading:
            return None
            
        voltage = reading['bus_voltage']
        
        # 12V batarya için voltaj-yüzde eğrisi
        if voltage >= 12.7:
            return 100.0
        elif voltage >= 12.5:
            return 90.0
        elif voltage >= 12.42:
            return 80.0
        elif voltage >= 12.32:
            return 70.0
        elif voltage >= 12.20:
            return 60.0
        elif voltage >= 12.06:
            return 50.0
        elif voltage >= 11.9:
            return 40.0
        elif voltage >= 11.75:
            return 30.0
        elif voltage >= 11.58:
            return 20.0
        elif voltage >= 11.31:
            return 10.0
        else:
            return 0.0
    
    def cleanup(self):
        """Sensörü temizle"""
        self.stop_reading()
