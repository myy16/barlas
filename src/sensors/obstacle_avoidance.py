"""
Engelden Kaçış Algoritması
JSN-SR04T ultrasonik sensörlerini kullanarak engelden kaçış
"""
import time
import math
from typing import Tuple, Optional
from .ultrasonic_manager import UltrasonicManager

class ObstacleAvoidance:
    """Engelden kaçış algoritması sınıfı"""
    
    def __init__(self):
        """ObstacleAvoidance başlatma"""
        self.ultrasonic = UltrasonicManager()
        
        # Kaçış parametreleri
        self.min_distance = 30.0  # Minimum güvenli mesafe (cm)
        self.max_distance = 200.0  # Maksimum ölçüm mesafesi (cm)
        self.danger_zone = 50.0   # Tehlike bölgesi mesafesi (cm)
        
        # Hareket parametreleri
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        
        # Başlat
        self.ultrasonic.start_measurements()
        
    def calculate_avoidance_vector(self) -> Tuple[float, float]:
        """Engelden kaçış vektörünü hesapla
        
        Returns:
            Tuple[float, float]: (linear_speed, angular_speed)
        """
        # Tüm mesafeleri al
        front_distances = self.ultrasonic.get_front_distances()
        rear_distances = self.ultrasonic.get_rear_distances()
        
        # Ön ve arka ortalama mesafeler
        avg_front = sum(front_distances) / len(front_distances)
        avg_rear = sum(rear_distances) / len(rear_distances)
        
        # Tehlike durumu kontrolü
        if min(front_distances) < self.danger_zone:
            # Ön bölgede yakın engel var
            return self._emergency_stop()
        
        # Engelden kaçış vektörünü hesapla
        linear_speed, angular_speed = self._calculate_avoidance(front_distances, rear_distances)
        
        return linear_speed, angular_speed
    
    def _emergency_stop(self) -> Tuple[float, float]:
        """Acil duruş komutu
        
        Returns:
            Tuple[float, float]: (0, yüksek_açısal_hız)
        """
        return 0.0, self.max_angular_speed
    
    def _calculate_avoidance(self, front_distances: list, rear_distances: list) -> Tuple[float, float]:
        """Detaylı engelden kaçış hesaplaması
        
        Args:
            front_distances (list): Ön sensör mesafeleri
            rear_distances (list): Arka sensör mesafeleri
            
        Returns:
            Tuple[float, float]: (linear_speed, angular_speed)
        """
        # Ağırlık vektörleri (merkeze daha fazla ağırlık)
        front_weights = [0.15, 0.35, 0.35, 0.15]  # Sağ ve sol kenarlar daha az önemli
        
        # Ağırlıklı mesafe vektörü hesapla
        weighted_distances = []
        for d, w in zip(front_distances, front_weights):
            # Mesafeyi normalizasyon
            normalized_dist = min(1.0, d / self.max_distance)
            weighted_distances.append(normalized_dist * w)
        
        # Sol ve sağ taraf karşılaştırması
        left_side = weighted_distances[0] + weighted_distances[1]
        right_side = weighted_distances[2] + weighted_distances[3]
        
        # Açısal hız hesapla (pozitif = sola dön)
        side_diff = right_side - left_side
        angular_speed = self.max_angular_speed * side_diff
        
        # Doğrusal hız hesapla (engele yaklaştıkça yavaşla)
        front_clearance = min(front_distances) / self.max_distance
        linear_speed = self.max_linear_speed * front_clearance
        
        # Hızları sınırla
        linear_speed = max(0.0, min(self.max_linear_speed, linear_speed))
        angular_speed = max(-self.max_angular_speed, 
                          min(self.max_angular_speed, angular_speed))
        
        return linear_speed, angular_speed
    
    def get_obstacle_status(self) -> dict:
        """Tüm sensörlerin engel durumunu döndür"""
        return self.ultrasonic.get_obstacle_status()
    
    def cleanup(self):
        """Kaynakları temizle"""
        self.ultrasonic.cleanup()
    
    def is_path_clear(self) -> bool:
        """Yolun açık olup olmadığını kontrol et"""
        front_distances = self.ultrasonic.get_front_distances()
        return all(d > self.min_distance for d in front_distances)
