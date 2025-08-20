"""
DS18B20 Sıcaklık Sensörü Yönetim Modülü
1-Wire protokolü ile sıcaklık ölçümü
"""
import os
import glob
import time
import threading
from typing import Optional, List

class TemperatureSensor:
    """DS18B20 sıcaklık sensörü yönetim sınıfı"""
    
    def __init__(self):
        """DS18B20 sensör yöneticisini başlat"""
        # 1-Wire arayüz dosya yolu
        self.device_path = '/sys/bus/w1/devices/'
        self.device_folder = None
        self.device_file = None
        
        # Okuma parametreleri
        self.reading = False
        self.read_thread = None
        self.lock = threading.Lock()
        
        # Son ölçüm değerleri
        self.last_temp = None
        self.last_read_time = 0
        
        # Çalışma parametreleri
        self.read_interval = 1.0  # saniye
        self.max_retries = 3
        
    def initialize(self) -> bool:
        """Sensörü başlat ve hazırla
        
        Returns:
            bool: Başlatma başarılı mı
        """
        try:
            # 1-Wire modülünü yükle
            os.system('modprobe w1-gpio')
            os.system('modprobe w1-therm')
            
            # Sensör dosyasını bul
            device_folders = glob.glob(self.device_path + '28*')
            if not device_folders:
                print("❌ DS18B20 sensörü bulunamadı!")
                return False
                
            self.device_folder = device_folders[0]
            self.device_file = self.device_folder + '/w1_slave'
            
            print(f"✅ DS18B20 sensörü bulundu: {self.device_folder}")
            return True
            
        except Exception as e:
            print(f"❌ Sensör başlatma hatası: {e}")
            return False
    
    def _read_temp_raw(self) -> List[str]:
        """Ham sıcaklık verisini oku"""
        try:
            with open(self.device_file, 'r') as f:
                lines = f.readlines()
            return lines
        except Exception:
            return []
    
    def read_temperature(self) -> Optional[float]:
        """Sıcaklık değerini oku ve Celsius olarak döndür
        
        Returns:
            float: Sıcaklık değeri (°C) veya None (hata durumunda)
        """
        if not self.device_file:
            return None
            
        # Birkaç deneme yap
        for _ in range(self.max_retries):
            lines = self._read_temp_raw()
            if len(lines) != 2:
                continue
                
            # CRC kontrolü
            if lines[0].strip()[-3:] != 'YES':
                time.sleep(0.2)
                continue
                
            # Sıcaklık değerini ayıkla
            equals_pos = lines[1].find('t=')
            if equals_pos != -1:
                temp_string = lines[1][equals_pos+2:]
                temp_c = float(temp_string) / 1000.0
                return round(temp_c, 2)
                
            time.sleep(0.2)
            
        return None
    
    def start_reading(self):
        """Sürekli okuma döngüsünü başlat"""
        if not self.reading:
            self.reading = True
            self.read_thread = threading.Thread(target=self._reading_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            print("✅ Sıcaklık okuma başlatıldı")
    
    def stop_reading(self):
        """Okuma döngüsünü durdur"""
        self.reading = False
        if self.read_thread:
            self.read_thread.join()
        print("⏹️ Sıcaklık okuma durduruldu")
    
    def _reading_loop(self):
        """Sürekli sıcaklık okuyan döngü"""
        while self.reading:
            temp = self.read_temperature()
            
            with self.lock:
                self.last_temp = temp
                self.last_read_time = time.time()
            
            time.sleep(self.read_interval)
    
    def get_last_temperature(self) -> Optional[float]:
        """Son okunan sıcaklık değerini döndür"""
        with self.lock:
            return self.last_temp
    
    def get_reading_age(self) -> float:
        """Son okumadan bu yana geçen süreyi döndür (saniye)"""
        with self.lock:
            return time.time() - self.last_read_time
    
    def cleanup(self):
        """Sensörü temizle"""
        self.stop_reading()
