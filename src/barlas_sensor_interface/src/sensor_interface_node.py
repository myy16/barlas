#!/usr/bin/env python3
"""
BARLAS Robot - ROS Sensör Arayüzü
Raspberry Pi'den gelen sensör verilerini ROS'a çeviren node
"""
import json
import rospy
import serial
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray, Header
from sensor_msgs.msg import LaserScan, Imu, Temperature
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SensorInterface:
    """Raspberry Pi'den gelen sensör verilerini ROS'a çevirme"""
    
    def __init__(self):
        """ROS node'unu ve yayıncıları başlat"""
        rospy.init_node('barlas_sensor_interface')
        
        # Parametre okuma
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        
        # Seri port bağlantısı
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate)
            rospy.loginfo(f"✅ Seri port bağlantısı başarılı: {self.serial_port}")
        except Exception as e:
            rospy.logerr(f"⚠️ Seri port bağlantı hatası: {e}")
            raise e
        
        # ROS yayıncıları
        self.ultrasonic_pub = rospy.Publisher(
            'sensors/ultrasonic', Float32MultiArray, queue_size=10)
        self.lidar_pub = rospy.Publisher(
            'sensors/lidar', LaserScan, queue_size=10)
        self.imu_pub = rospy.Publisher(
            'sensors/imu', Imu, queue_size=10)
        self.temp_pub = rospy.Publisher(
            'sensors/temperature', Temperature, queue_size=10)
        self.battery_pub = rospy.Publisher(
            'system/battery', DiagnosticArray, queue_size=10)
        
        rospy.loginfo("✅ ROS node başlatıldı")
    
    def publish_ultrasonics(self, distances):
        """Ultrasonik sensör verilerini yayınla"""
        msg = Float32MultiArray()
        msg.data = distances
        self.ultrasonic_pub.publish(msg)
    
    def publish_lidar(self, lidar_data):
        """LIDAR tarama verilerini yayınla"""
        if not lidar_data:
            return
            
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_laser"
        
        # LIDAR verilerini doldur
        msg.angle_min = 0.0
        msg.angle_max = 2 * np.pi
        msg.angle_increment = (2 * np.pi) / len(lidar_data)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.ranges = lidar_data
        
        self.lidar_pub.publish(msg)
    
    def publish_imu(self, imu_data):
        """IMU verilerini yayınla"""
        if not imu_data:
            return
            
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_imu"
        
        # IMU verilerini doldur
        if 'orientation' in imu_data:
            msg.orientation.x = imu_data['orientation'][0]
            msg.orientation.y = imu_data['orientation'][1]
            msg.orientation.z = imu_data['orientation'][2]
            msg.orientation.w = imu_data['orientation'][3]
            
        if 'angular_velocity' in imu_data:
            msg.angular_velocity.x = imu_data['angular_velocity'][0]
            msg.angular_velocity.y = imu_data['angular_velocity'][1]
            msg.angular_velocity.z = imu_data['angular_velocity'][2]
            
        if 'linear_acceleration' in imu_data:
            msg.linear_acceleration.x = imu_data['linear_acceleration'][0]
            msg.linear_acceleration.y = imu_data['linear_acceleration'][1]
            msg.linear_acceleration.z = imu_data['linear_acceleration'][2]
        
        self.imu_pub.publish(msg)
    
    def publish_temperature(self, temp):
        """Sıcaklık verisini yayınla"""
        msg = Temperature()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_temp"
        msg.temperature = temp
        msg.variance = 0.0
        
        self.temp_pub.publish(msg)
    
    def publish_battery(self, voltage, current):
        """Batarya durumunu yayınla"""
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        
        status = DiagnosticStatus()
        status.name = "Robot Battery"
        status.hardware_id = "battery_0"
        
        # Batarya durumunu belirle
        if voltage < 10.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "Critical Battery Level"
        elif voltage < 10.5:
            status.level = DiagnosticStatus.WARN
            status.message = "Low Battery"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Battery OK"
        
        # Değerleri ekle
        status.values = [
            KeyValue(key="Voltage", value=f"{voltage:.2f}"),
            KeyValue(key="Current", value=f"{current:.2f}")
        ]
        
        msg.status = [status]
        self.battery_pub.publish(msg)
    
    def run(self):
        """Ana çalışma döngüsü"""
        rate = rospy.Rate(50)  # 50Hz
        
        while not rospy.is_shutdown():
            try:
                # Seri porttan veri oku
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8').strip()
                    data = json.loads(line)
                    
                    # Sensör verilerini ROS'a çevir ve yayınla
                    if 'ultrasonics' in data:
                        self.publish_ultrasonics(data['ultrasonics'])
                    if 'lidar' in data:
                        self.publish_lidar(data['lidar'])
                    if 'imu' in data:
                        self.publish_imu(data['imu'])
                    if 'temperature' in data:
                        self.publish_temperature(data['temperature'])
                    if 'battery' in data:
                        self.publish_battery(
                            data['battery']['voltage'],
                            data['battery']['current']
                        )
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"⚠️ Veri okuma/yayınlama hatası: {e}")
                rospy.sleep(1.0)
    
    def shutdown(self):
        """Node kapatılırken temizlik"""
        self.serial.close()
        rospy.loginfo("⏹️ Sensör arayüzü durduruldu")

if __name__ == '__main__':
    try:
        interface = SensorInterface()
        rospy.on_shutdown(interface.shutdown)
        interface.run()
    except rospy.ROSInterruptException:
        pass
