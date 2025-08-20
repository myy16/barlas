#!/usr/bin/env python3
"""
BARLAS Robot - YDLIDAR X4 ROS Node'u
2D LIDAR sensörünün ROS entegrasyonu
"""
import rospy
from sensor_msgs.msg import LaserScan
from ydlidar import YDLidarX4
import numpy as np
import threading
import time

class LidarNode:
    """YDLIDAR X4 node sınıfı"""
    
    def __init__(self):
        """LidarNode başlatma"""
        # ROS node başlat
        rospy.init_node('lidar_node', anonymous=True)
        
        # ROS yayıncısı
        self.scan_pub = rospy.Publisher('/barlas/sensors/lidar/scan', 
                                      LaserScan, queue_size=10)
        
        # Parametreleri al
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.frame_id = rospy.get_param('~frame_id', 'laser')
        self.scan_frequency = rospy.get_param('~scan_frequency', 8)  # Hz
        self.sample_rate = rospy.get_param('~sample_rate', 9)       # kHz
        
        # LIDAR sensörü
        self.lidar = None
        self.scanning = False
        self.scan_thread = None
        
        # LaserScan mesajı şablonu
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.frame_id
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = 2 * np.pi
        self.scan_msg.range_min = 0.12  # 12cm
        self.scan_msg.range_max = 10.0   # 10m
        
        rospy.loginfo("✅ LIDAR node'u başlatıldı")
    
    def connect(self) -> bool:
        """LIDAR'a bağlan ve başlat
        
        Returns:
            bool: Bağlantı başarılı mı
        """
        try:
            # LIDAR'ı başlat
            self.lidar = YDLidarX4(self.port)
            self.lidar.connect()
            
            # Parametreleri ayarla
            self.lidar.set_scan_frequency(self.scan_frequency)
            self.lidar.set_sample_rate(self.sample_rate)
            
            # Açısal çözünürlüğü hesapla
            points_per_scan = self.sample_rate * 1000 / self.scan_frequency
            self.scan_msg.angle_increment = 2 * np.pi / points_per_scan
            self.scan_msg.time_increment = 1.0 / (self.scan_frequency * points_per_scan)
            self.scan_msg.scan_time = 1.0 / self.scan_frequency
            
            rospy.loginfo(f"✅ LIDAR bağlandı: {self.port}")
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ LIDAR bağlantı hatası: {e}")
            return False
    
    def start_scanning(self):
        """Tarama döngüsünü başlat"""
        if not self.scanning:
            self.scanning = True
            self.scan_thread = threading.Thread(target=self._scanning_loop)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            rospy.loginfo("✅ LIDAR tarama başlatıldı")
    
    def stop_scanning(self):
        """Tarama döngüsünü durdur"""
        self.scanning = False
        if self.scan_thread:
            self.scan_thread.join()
        rospy.loginfo("⏹️ LIDAR tarama durduruldu")
    
    def _scanning_loop(self):
        """Sürekli tarama yapan döngü"""
        while self.scanning and not rospy.is_shutdown():
            try:
                # Yeni tarama al
                scan = self.lidar.get_scan()
                
                # LaserScan mesajını hazırla
                self.scan_msg.header.stamp = rospy.Time.now()
                self.scan_msg.ranges = scan.distances
                self.scan_msg.intensities = scan.intensities
                
                # Mesajı yayınla
                self.scan_pub.publish(self.scan_msg)
                
            except Exception as e:
                rospy.logerr(f"⚠️ Tarama hatası: {e}")
                rospy.sleep(1.0)
    
    def run(self):
        """Ana çalışma döngüsü"""
        try:
            if self.connect():
                self.start_scanning()
                rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.stop_scanning()
            if self.lidar:
                self.lidar.disconnect()
            rospy.loginfo("⏹️ LIDAR node'u kapatıldı")

if __name__ == '__main__':
    try:
        node = LidarNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
