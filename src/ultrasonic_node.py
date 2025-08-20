#!/usr/bin/env python3
"""
BARLAS Robot - Ultrasonik Sensör ROS Node'u
8 adet JSN-SR04T sensörünün ROS entegrasyonu
"""
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

class UltrasonicNode:
    """Ultrasonik sensör node sınıfı"""
    
    # Sensör pin konfigürasyonu - (trigger_pin, echo_pin)
    SENSOR_PINS = {
        1: (17, 27),  # Ön sol
        2: (22, 23),  # Ön orta-sol
        3: (24, 25),  # Ön orta-sağ
        4: (5, 6),    # Ön sağ
        5: (13, 19),  # Arka sol
        6: (20, 21),  # Arka orta-sol
        7: (12, 16),  # Arka orta-sağ
        8: (26, 18)   # Arka sağ
    }
    
    def __init__(self):
        """UltrasonicNode başlatma"""
        # ROS node başlat
        rospy.init_node('ultrasonic_node', anonymous=True)
        
        # ROS yayıncıları
        self.distances_pub = rospy.Publisher('/barlas/sensors/ultrasonic/distances', 
                                           Float32MultiArray, queue_size=10)
        self.nearest_pub = rospy.Publisher('/barlas/sensors/ultrasonic/nearest', 
                                         PointStamped, queue_size=10)
        
        # Parametreleri al
        self.rate = rospy.Rate(10)  # 10 Hz
        self.min_distance = rospy.get_param('~min_distance', 0.02)  # 2cm
        self.max_distance = rospy.get_param('~max_distance', 4.0)   # 4m
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self._setup_gpio()
        
        rospy.loginfo("✅ Ultrasonik sensör node'u başlatıldı")
    
    def _setup_gpio(self):
        """GPIO pinlerini ayarla"""
        for trigger_pin, echo_pin in self.SENSOR_PINS.values():
            GPIO.setup(trigger_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN)
            GPIO.output(trigger_pin, False)
    
    def measure_distance(self, trigger_pin: int, echo_pin: int) -> float:
        """Tek bir sensörden mesafe ölç
        
        Args:
            trigger_pin (int): Trigger pin numarası
            echo_pin (int): Echo pin numarası
            
        Returns:
            float: Ölçülen mesafe (metre)
        """
        # Trigger pulse gönder
        GPIO.output(trigger_pin, True)
        rospy.sleep(0.00001)  # 10 microseconds
        GPIO.output(trigger_pin, False)
        
        # Echo pulse'ı bekle
        start_time = rospy.get_time()
        timeout = start_time + 0.1  # 100ms timeout
        
        # Echo yüksek seviyeyi bekle
        while GPIO.input(echo_pin) == 0:
            start_time = rospy.get_time()
            if start_time > timeout:
                return float('inf')
        
        # Echo düşük seviyeyi bekle
        stop_time = rospy.get_time()
        while GPIO.input(echo_pin) == 1:
            stop_time = rospy.get_time()
            if stop_time > timeout:
                return float('inf')
        
        # Mesafe hesapla (metre)
        duration = stop_time - start_time
        distance = (duration * 343.0) / 2  # Ses hızı 343 m/s
        
        # Mesafeyi sınırla
        distance = min(max(distance, self.min_distance), self.max_distance)
        
        return distance
    
    def get_all_distances(self) -> list:
        """Tüm sensörlerden mesafe ölç
        
        Returns:
            list: 8 sensörün mesafe değerleri (metre)
        """
        distances = []
        for sensor_id in range(1, 9):
            trigger_pin, echo_pin = self.SENSOR_PINS[sensor_id]
            distance = self.measure_distance(trigger_pin, echo_pin)
            distances.append(distance)
        return distances
    
    def find_nearest_obstacle(self, distances: list) -> tuple:
        """En yakın engeli bul
        
        Args:
            distances (list): Mesafe listesi
            
        Returns:
            tuple: (sensor_id, distance) - En yakın engelin ID'si ve mesafesi
        """
        min_distance = float('inf')
        min_sensor_id = None
        
        for i, distance in enumerate(distances):
            if distance < min_distance:
                min_distance = distance
                min_sensor_id = i + 1
        
        return min_sensor_id, min_distance
    
    def run(self):
        """Ana çalışma döngüsü"""
        try:
            while not rospy.is_shutdown():
                # Tüm mesafeleri ölç
                distances = self.get_all_distances()
                
                # Mesafe mesajı yayınla
                msg = Float32MultiArray()
                msg.data = distances
                self.distances_pub.publish(msg)
                
                # En yakın engeli bul ve yayınla
                nearest_id, nearest_dist = self.find_nearest_obstacle(distances)
                if nearest_id is not None:
                    msg = PointStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = f"ultrasonic_{nearest_id}"
                    msg.point.x = nearest_dist
                    self.nearest_pub.publish(msg)
                
                # Hız kontrolü
                self.rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
        finally:
            GPIO.cleanup()
            rospy.loginfo("⏹️ Ultrasonik sensör node'u kapatıldı")

if __name__ == '__main__':
    try:
        node = UltrasonicNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
