"""
MPU9250 IMU Sensörü Yönetim Modülü
9 eksenli IMU sensörü (ivme, jiroskop, manyetometre)
"""
import time
import threading
import numpy as np
from typing import Dict, Optional, Tuple
from mpu9250_i2c import MPU9250

class IMUSensor:
    """MPU9250 IMU sensörü yönetim sınıfı"""
    
    def __init__(self, i2c_addr: int = 0x68):
        """
        Args:
            i2c_addr (int): I2C adres (varsayılan: 0x68)
        """
        self.i2c_addr = i2c_addr
        self.sensor = None
        
        # Okuma parametreleri
        self.reading = False
        self.read_thread = None
        self.lock = threading.Lock()
        
        # Son ölçüm değerleri
        self.last_reading = None
        self.last_read_time = 0
        
        # Çalışma parametreleri
        self.read_interval = 0.01  # 100Hz
        self.calibration = None
        
    def initialize(self) -> bool:
        """Sensörü başlat ve hazırla
        
        Returns:
            bool: Başlatma başarılı mı
        """
        try:
            # MPU9250 sensörünü başlat
            self.sensor = MPU9250(address=self.i2c_addr)
            
            # Sensörü kalibre et
            self.calibrate()
            
            print(f"✅ MPU9250 sensörü başlatıldı (0x{self.i2c_addr:02X})")
            return True
            
        except Exception as e:
            print(f"❌ Sensör başlatma hatası: {e}")
            return False
    
    def calibrate(self, samples: int = 100):
        """Sensörü kalibre et
        
        Args:
            samples (int): Kalibrasyon için örnek sayısı
        """
        print("📊 IMU kalibrasyonu başlıyor...")
        
        # Kalibrasyon için veri topla
        accel_bias = np.zeros(3)
        gyro_bias = np.zeros(3)
        mag_bias = np.zeros(3)
        
        for _ in range(samples):
            reading = self._read_raw()
            if reading:
                accel_bias += np.array(reading['accelerometer'])
                gyro_bias += np.array(reading['gyroscope'])
                mag_bias += np.array(reading['magnetometer'])
            time.sleep(0.01)
        
        # Ortalama sapmaları hesapla
        self.calibration = {
            'accel_bias': accel_bias / samples,
            'gyro_bias': gyro_bias / samples,
            'mag_bias': mag_bias / samples
        }
        
        print("✅ IMU kalibrasyonu tamamlandı")
    
    def _read_raw(self) -> Optional[Dict[str, Tuple[float, float, float]]]:
        """Ham sensör verilerini oku"""
        if not self.sensor:
            return None
            
        try:
            return {
                'accelerometer': self.sensor.readAccel(),    # g
                'gyroscope': self.sensor.readGyro(),         # derece/s
                'magnetometer': self.sensor.readMagnet()     # uT
            }
        except:
            return None
    
    def read_measurements(self) -> Optional[Dict[str, Tuple[float, float, float]]]:
        """Kalibre edilmiş ölçümleri döndür"""
        raw = self._read_raw()
        if not raw or not self.calibration:
            return raw
            
        # Kalibrasyon uygula
        calibrated = {
            'accelerometer': tuple(np.array(raw['accelerometer']) - 
                                 self.calibration['accel_bias']),
            'gyroscope': tuple(np.array(raw['gyroscope']) - 
                             self.calibration['gyro_bias']),
            'magnetometer': tuple(np.array(raw['magnetometer']) - 
                                self.calibration['mag_bias'])
        }
        
        return calibrated
    
    def start_reading(self):
        """Sürekli okuma döngüsünü başlat"""
        if not self.reading:
            self.reading = True
            self.read_thread = threading.Thread(target=self._reading_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            print("✅ IMU ölçümleri başlatıldı")
    
    def stop_reading(self):
        """Okuma döngüsünü durdur"""
        self.reading = False
        if self.read_thread:
            self.read_thread.join()
        print("⏹️ IMU ölçümleri durduruldu")
    
    def _reading_loop(self):
        """Sürekli ölçüm yapan döngü"""
        while self.reading:
            measurements = self.read_measurements()
            
            with self.lock:
                self.last_reading = measurements
                self.last_read_time = time.time()
            
            time.sleep(self.read_interval)
    
    def get_last_reading(self) -> Optional[Dict[str, Tuple[float, float, float]]]:
        """Son ölçüm değerlerini döndür"""
        with self.lock:
            return self.last_reading
    
    def get_reading_age(self) -> float:
        """Son ölçümden bu yana geçen süreyi döndür (saniye)"""
        with self.lock:
            return time.time() - self.last_read_time
    
    def get_orientation(self) -> Optional[Dict[str, float]]:
        """Euler açılarını hesapla (derece)
        
        Returns:
            Dict with:
            - 'roll': x ekseni etrafında dönüş
            - 'pitch': y ekseni etrafında dönüş
            - 'yaw': z ekseni etrafında dönüş
        """
        reading = self.get_last_reading()
        if not reading:
            return None
            
        try:
            # İvmeölçer verilerinden roll ve pitch hesapla
            ax, ay, az = reading['accelerometer']
            
            roll = np.arctan2(ay, az) * 180.0 / np.pi
            pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az)) * 180.0 / np.pi
            
            # Manyetometreden yaw hesapla
            mx, my, mz = reading['magnetometer']
            
            yaw = np.arctan2(my, mx) * 180.0 / np.pi
            if yaw < 0:
                yaw += 360.0
                
            return {
                'roll': round(roll, 2),
                'pitch': round(pitch, 2),
                'yaw': round(yaw, 2)
            }
            
        except:
            return None
    
    def cleanup(self):
        """Sensörü temizle"""
        self.stop_reading()
