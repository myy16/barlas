"""
YDLIDAR X4 Sensör Yönetim Modülü
2D LIDAR sensörünün kontrolü ve veri işleme
"""
import os
import time
import threading
import numpy as np
from typing import List, Dict, Optional, Tuple
from ydlidar import YDLidarX4

class LidarManager:
    """YDLIDAR X4 sensör yönetim sınıfı"""
    
    def __init__(self, port: str = "/dev/ttyUSB0"):
        """
        Args:
            port (str): LIDAR'ın bağlı olduğu seri port
        """
        self.port = port
        self.lidar = YDLidarX4(port)
        self.scanning = False
        self.scan_thread = None
        self.lock = threading.Lock()
        
        # Tarama verisi
        self.last_scan = None
        self.last_scan_time = 0
        
        # LIDAR parametreleri
        self.scan_frequency = 8  # Hz
        self.sample_rate = 9     # kHz
        self.angle_min = 0       # derece
        self.angle_max = 359     # derece
        
    def connect(self) -> bool:
        """LIDAR'a bağlan ve başlat
        
        Returns:
            bool: Bağlantı başarılı mı
        """
        try:
            # LIDAR'ı başlat
            self.lidar.connect()
            
            # Parametreleri ayarla
            self.lidar.set_scan_frequency(self.scan_frequency)
            self.lidar.set_sample_rate(self.sample_rate)
            
            print(f"✅ YDLIDAR X4 bağlandı: {self.port}")
            print(f"📊 Tarama frekansı: {self.scan_frequency} Hz")
            print(f"📊 Örnekleme hızı: {self.sample_rate} kHz")
            return True
            
        except Exception as e:
            print(f"❌ LIDAR bağlantı hatası: {e}")
            return False
    
    def start_scanning(self):
        """Sürekli tarama döngüsünü başlat"""
        if not self.scanning:
            self.scanning = True
            self.scan_thread = threading.Thread(target=self._scanning_loop)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            print("✅ LIDAR tarama başlatıldı")
    
    def stop_scanning(self):
        """Tarama döngüsünü durdur"""
        self.scanning = False
        if self.scan_thread:
            self.scan_thread.join()
        print("⏹️ LIDAR tarama durduruldu")
    
    def _scanning_loop(self):
        """Sürekli tarama yapan döngü"""
        while self.scanning:
            try:
                # Yeni tarama al
                scan = self.lidar.get_scan()
                
                with self.lock:
                    self.last_scan = scan
                    self.last_scan_time = time.time()
                
                # Tarama frekansına göre bekle
                time.sleep(1.0 / self.scan_frequency)
                
            except Exception as e:
                print(f"⚠️ Tarama hatası: {e}")
                time.sleep(1.0)
    
    def get_latest_scan(self) -> Optional[Dict[str, np.ndarray]]:
        """En son tarama verisini döndür
        
        Returns:
            Dict with:
            - 'angles': Açı değerleri (derece)
            - 'distances': Mesafe değerleri (metre)
            - 'intensities': Sinyal güçleri
        """
        with self.lock:
            if self.last_scan is None:
                return None
                
            return {
                'angles': np.array(self.last_scan.angles),
                'distances': np.array(self.last_scan.distances),
                'intensities': np.array(self.last_scan.intensities)
            }
    
    def get_sector_scan(self, start_angle: float, end_angle: float) -> Optional[Dict[str, np.ndarray]]:
        """Belirli bir açı aralığındaki tarama verisini döndür
        
        Args:
            start_angle (float): Başlangıç açısı (derece)
            end_angle (float): Bitiş açısı (derece)
            
        Returns:
            Dict: Seçilen bölgenin tarama verisi
        """
        scan = self.get_latest_scan()
        if scan is None:
            return None
            
        # Açı aralığındaki noktaları seç
        mask = (scan['angles'] >= start_angle) & (scan['angles'] <= end_angle)
        
        return {
            'angles': scan['angles'][mask],
            'distances': scan['distances'][mask],
            'intensities': scan['intensities'][mask]
        }
    
    def get_obstacles_in_range(self, min_distance: float, max_distance: float) -> List[Tuple[float, float]]:
        """Belirli mesafe aralığındaki engelleri tespit et
        
        Args:
            min_distance (float): Minimum mesafe (metre)
            max_distance (float): Maksimum mesafe (metre)
            
        Returns:
            List[Tuple[float, float]]: (açı, mesafe) formatında engel listesi
        """
        scan = self.get_latest_scan()
        if scan is None:
            return []
            
        # Mesafe aralığındaki noktaları bul
        mask = (scan['distances'] >= min_distance) & (scan['distances'] <= max_distance)
        
        obstacles = []
        for angle, distance in zip(scan['angles'][mask], scan['distances'][mask]):
            obstacles.append((angle, distance))
            
        return obstacles
    
    def cleanup(self):
        """LIDAR'ı temizle ve kapat"""
        self.stop_scanning()
        if self.lidar:
            self.lidar.disconnect()
        print("🧹 LIDAR temizlendi")
