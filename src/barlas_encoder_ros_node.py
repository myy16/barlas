#!/usr/bin/env python3
"""
BARLAS Encoder ROS Node
Arduino'dan encoder verilerini okuyup ROS topic'lere publish eder
Odometry hesaplaması ve tf yayını yapar
"""

import rospy
import tf2_ros
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# Arduino controller import
try:
    import sys
    import os
    sys.path.append(os.path.join(os.path.dirname(__file__), 'dart_laser_system'))
    from arduino_controller_fixed import BarlasVehicleController
    ARDUINO_AVAILABLE = True
    print("✅ BARLAS Arduino Controller yüklendi")
except ImportError:
    ARDUINO_AVAILABLE = False
    print("⚠️ BARLAS Arduino Controller bulunamadı")

class BarlasEncoderROSNode:
    """
    BARLAS Encoder ROS Node
    - Arduino'dan encoder verilerini oku
    - Odometry hesapla ve yayınla  
    - TF transformlarını yayınla
    - Wheel velocities publish et
    """
    
    def __init__(self):
        """ROS node başlatma"""
        rospy.init_node('barlas_encoder_node')
        
        print("🎯 [BARLAS Encoder] ROS Node başlatılıyor...")
        
        # Arduino bağlantısı
        self.arduino_port = rospy.get_param('~arduino_port', None)
        self.arduino = None
        self.initialize_arduino()
        
        # Robot parametreleri
        self.wheel_base = rospy.get_param('~wheel_base', 0.52)  # Tekerlek arası mesafe (metre)
        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.20)  # Tekerlek çapı (metre) 
        self.encoder_resolution = rospy.get_param('~encoder_resolution', 1400)  # Encoder pulse/revolution
        
        # Hesaplama parametreleri
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.pulse_per_meter = self.encoder_resolution / self.wheel_circumference
        
        print(f"[BARLAS Encoder] Robot parametreleri:")
        print(f"  └─ Wheel base: {self.wheel_base:.2f}m")
        print(f"  └─ Wheel diameter: {self.wheel_diameter:.2f}m") 
        print(f"  └─ Encoder resolution: {self.encoder_resolution} pulse/rev")
        print(f"  └─ Pulse per meter: {self.pulse_per_meter:.1f}")
        
        # Encoder durumu
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.last_time = rospy.Time.now()
        
        # Odometry durumu
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # ROS Publishers
        self.encoder_pub = rospy.Publisher('/barlas/encoders/raw', Float32MultiArray, queue_size=10)
        self.odom_pub = rospy.Publisher('/barlas/odom', Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher('/barlas/joint_states', JointState, queue_size=10)
        self.wheel_vel_pub = rospy.Publisher('/barlas/wheel_velocities', Twist, queue_size=10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Timer - encoder okuma (20Hz)
        self.encoder_timer = rospy.Timer(rospy.Duration(0.05), self.read_and_publish_encoders)
        
        print("✅ [BARLAS Encoder] ROS Node hazır!")
        print("📊 Published topics:")
        print("  ├─ /barlas/encoders/raw (Float32MultiArray) - Ham encoder değerleri")
        print("  ├─ /barlas/odom (Odometry) - Robot odometry")
        print("  ├─ /barlas/joint_states (JointState) - Tekerlek açıları")
        print("  └─ /barlas/wheel_velocities (Twist) - Tekerlek hızları")
        print("🗺️ TF frames: odom -> base_link")
        
    def initialize_arduino(self):
        """Arduino bağlantısını kur"""
        if not ARDUINO_AVAILABLE:
            print("[BARLAS Encoder] ⚠️ Arduino controller yok - simülasyon modu")
            return
            
        try:
            # Manual port öncelikli
            if self.arduino_port:
                print(f"[BARLAS Encoder] Manuel port deneniyor: {self.arduino_port}")
                self.arduino = BarlasVehicleController(port=self.arduino_port)
                if self.arduino.connect():
                    print(f"[BARLAS Encoder] ✅ Arduino bağlandı: {self.arduino_port}")
                    return
                else:
                    self.arduino = None
                    
            # Otomatik port tarama
            import sys
            if sys.platform.startswith('linux'):
                ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
            else:
                ports = ['COM7', 'COM6', 'COM5', 'COM8']
                
            for port in ports:
                try:
                    print(f"[BARLAS Encoder] Arduino bağlantısı deneniyor: {port}")
                    self.arduino = BarlasVehicleController(port=port)
                    if self.arduino.connect():
                        print(f"[BARLAS Encoder] ✅ Arduino bağlandı: {port}")
                        return
                    else:
                        self.arduino = None
                except Exception as e:
                    print(f"[BARLAS Encoder] Port hatası {port}: {e}")
                    continue
                    
            print("[BARLAS Encoder] ❌ Arduino bağlantısı başarısız!")
            
        except Exception as e:
            print(f"[BARLAS Encoder] Arduino başlatma hatası: {e}")
            self.arduino = None
    
    def read_and_publish_encoders(self, event):
        """Encoder değerlerini oku ve publish et"""
        try:
            current_time = rospy.Time.now()
            
            # Arduino'dan encoder oku
            if self.arduino:
                encoder_data = self.arduino.read_encoders()
                if encoder_data:
                    encoder_left, encoder_right = encoder_data
                else:
                    # Okuma hatası - önceki değerleri koru
                    encoder_left = self.last_encoder_left
                    encoder_right = self.last_encoder_right
            else:
                # Simülasyon - test verileri
                import time
                t = time.time()
                encoder_left = int(1000 * math.sin(t * 0.1))
                encoder_right = int(1000 * math.cos(t * 0.1))
            
            # Ham encoder verilerini publish et
            raw_msg = Float32MultiArray()
            raw_msg.data = [float(encoder_left), float(encoder_right)]
            self.encoder_pub.publish(raw_msg)
            
            # Odometry hesapla ve publish et
            self.calculate_and_publish_odometry(encoder_left, encoder_right, current_time)
            
            # Tekerlek durumlarını publish et
            self.publish_wheel_states(encoder_left, encoder_right, current_time)
            
            # Değerleri güncelle
            self.last_encoder_left = encoder_left
            self.last_encoder_right = encoder_right
            self.last_time = current_time
            
        except Exception as e:
            rospy.logwarn(f"[BARLAS Encoder] Encoder okuma hatası: {e}")
    
    def calculate_and_publish_odometry(self, encoder_left, encoder_right, current_time):
        """Odometry hesapla ve publish et"""
        try:
            dt = (current_time - self.last_time).to_sec()
            if dt <= 0:
                return
                
            # Encoder farkını hesapla
            delta_left = encoder_left - self.last_encoder_left
            delta_right = encoder_right - self.last_encoder_right
            
            # Mesafeye çevir (metre)
            distance_left = delta_left / self.pulse_per_meter
            distance_right = delta_right / self.pulse_per_meter
            
            # Robot kinematiği - differential drive
            distance_center = (distance_left + distance_right) / 2.0
            delta_theta = (distance_right - distance_left) / self.wheel_base
            
            # Pozisyon güncelle
            if abs(delta_theta) < 1e-6:
                # Düz hareket
                delta_x = distance_center * math.cos(self.theta)
                delta_y = distance_center * math.sin(self.theta)
            else:
                # Döner hareket
                radius = distance_center / delta_theta
                delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
                delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Theta'yı -pi ile pi arasında tut
            while self.theta > math.pi:
                self.theta -= 2.0 * math.pi
            while self.theta < -math.pi:
                self.theta += 2.0 * math.pi
            
            # Hızları hesapla
            vx = distance_center / dt
            vtheta = delta_theta / dt
            
            # Odometry mesajını oluştur
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            
            # Pozisyon
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            
            # Quaternion oryantasyon
            odom_quat = tf2_ros.transformations.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = odom_quat[0]
            odom_msg.pose.pose.orientation.y = odom_quat[1]
            odom_msg.pose.pose.orientation.z = odom_quat[2]
            odom_msg.pose.pose.orientation.w = odom_quat[3]
            
            # Hız
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = vtheta
            
            # Covariance (güvenirlik matrisi)
            odom_msg.pose.covariance[0] = 0.01   # x
            odom_msg.pose.covariance[7] = 0.01   # y  
            odom_msg.pose.covariance[35] = 0.1   # theta
            odom_msg.twist.covariance[0] = 0.1   # vx
            odom_msg.twist.covariance[35] = 0.2  # vtheta
            
            # Publish et
            self.odom_pub.publish(odom_msg)
            
            # TF transform yayınla
            self.publish_tf_transform(current_time, odom_quat)
            
            # Debug info (1Hz)
            if int(current_time.to_sec()) % 1 == 0:
                rospy.loginfo(f"[BARLAS Encoder] Pos: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.theta):.1f}°) Vel: ({vx:.2f}m/s, {math.degrees(vtheta):.1f}°/s)")
                
        except Exception as e:
            rospy.logwarn(f"[BARLAS Encoder] Odometry hesaplama hatası: {e}")
    
    def publish_tf_transform(self, current_time, odom_quat):
        """TF transform yayınla"""
        try:
            transform = TransformStamped()
            transform.header.stamp = current_time
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.x = odom_quat[0]
            transform.transform.rotation.y = odom_quat[1]
            transform.transform.rotation.z = odom_quat[2]
            transform.transform.rotation.w = odom_quat[3]
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            rospy.logwarn(f"[BARLAS Encoder] TF publish hatası: {e}")
    
    def publish_wheel_states(self, encoder_left, encoder_right, current_time):
        """Tekerlek durumlarını publish et"""
        try:
            dt = (current_time - self.last_time).to_sec()
            
            # Tekerlek açıları (radyan)
            wheel_angle_left = 2.0 * math.pi * encoder_left / self.encoder_resolution
            wheel_angle_right = 2.0 * math.pi * encoder_right / self.encoder_resolution
            
            # Tekerlek hızları (rad/s)
            if dt > 0:
                delta_left = encoder_left - self.last_encoder_left
                delta_right = encoder_right - self.last_encoder_right
                
                wheel_vel_left = 2.0 * math.pi * delta_left / (self.encoder_resolution * dt)
                wheel_vel_right = 2.0 * math.pi * delta_right / (self.encoder_resolution * dt)
            else:
                wheel_vel_left = 0.0
                wheel_vel_right = 0.0
            
            # Joint States mesajı
            joint_msg = JointState()
            joint_msg.header.stamp = current_time
            joint_msg.name = ["left_wheel_joint", "right_wheel_joint"]
            joint_msg.position = [wheel_angle_left, wheel_angle_right]
            joint_msg.velocity = [wheel_vel_left, wheel_vel_right]
            joint_msg.effort = [0.0, 0.0]
            
            self.joint_pub.publish(joint_msg)
            
            # Wheel velocities mesajı (linear velocities m/s)
            wheel_vel_msg = Twist()
            wheel_vel_msg.linear.x = wheel_vel_left * self.wheel_diameter / 2.0   # Sol tekerlek
            wheel_vel_msg.linear.y = wheel_vel_right * self.wheel_diameter / 2.0  # Sağ tekerlek
            wheel_vel_msg.angular.z = (wheel_vel_right - wheel_vel_left) * self.wheel_diameter / (2.0 * self.wheel_base)  # Angular velocity
            
            self.wheel_vel_pub.publish(wheel_vel_msg)
            
        except Exception as e:
            rospy.logwarn(f"[BARLAS Encoder] Wheel states publish hatası: {e}")

def main():
    """Ana fonksiyon"""
    try:
        encoder_node = BarlasEncoderROSNode()
        
        print("🎯 BARLAS Encoder ROS Node başlatıldı!")
        print("📊 Encoder verilerini Arduino'dan okuyor...")
        print("🗺️ Odometry hesaplıyor ve TF yayınlıyor...")
        print("⚡ 20Hz refresh rate")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("🛑 [BARLAS Encoder] Node kapatılıyor...")
    except Exception as e:
        print(f"❌ [BARLAS Encoder] HATA: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
