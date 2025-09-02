#!/usr/bin/env python3
"""
BARLAS ROS Bridge - Hibrit Sistem Entegrasyonu
WSL2 ROS Noetic + Windows Mission Planner + BARLAS Dart Laser Sistemi
"""

import rospy
from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import Twist, Point, PointStamped
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode

import threading
import json
import serial
import time
import math

class BARLASROSBridge:
    """
    Ana ROS köprüsü - tüm sistemleri bağlayan merkezi hub
    Mission Planner <-> ROS <-> Dart Laser <-> Sensörler
    """
    
    def __init__(self):
        rospy.init_node('barlas_ros_bridge', anonymous=True)
        
        print("🎯 [BARLAS Bridge] Sistem başlatılıyor...")
        
        # === MAVROS Entegrasyonu ===
        self.mavros_state = None
        self.mavros_armed = False
        self.current_mode = "MANUAL"
        
        # MAVROS subscribers
        self.mavros_state_sub = rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)
        
        # MAVROS publishers  
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        
        # MAVROS services
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # === Dart Laser Sistemi Entegrasyonu ===
        # Dart laser topic'leri (mevcut sisteminizden)
        self.dart_detection_sub = rospy.Subscriber('/barlas/dart_laser/dart_detected', PointStamped, self.dart_detected_callback)
        self.dart_laser_status_sub = rospy.Subscriber('/barlas/dart_laser/laser_status', Bool, self.laser_status_callback)
        
        # Servo kontrol publisher'ları
        self.servo_position_pub = rospy.Publisher('/barlas/servo/position_command', Float32MultiArray, queue_size=1)
        self.servo_laser_pub = rospy.Publisher('/barlas/servo/laser_command', Bool, queue_size=1)
        self.targeting_enable_pub = rospy.Publisher('/barlas/dart_laser/targeting_enable', Bool, queue_size=1)
        
        # === Sensör Entegrasyonu ===
        # Raspberry Pi sensör verileri
        self.sensor_data_sub = rospy.Subscriber('/barlas/sensors/all_data', String, self.sensor_data_callback)
        self.ultrasonic_sub = rospy.Subscriber('/barlas/sensors/ultrasonic', LaserScan, self.ultrasonic_callback)
        self.imu_sub = rospy.Subscriber('/barlas/sensors/imu', Imu, self.imu_callback)
        
        # === Encoder Entegrasyonu ===
        # Arduino encoder verileri
        self.encoder_sub = rospy.Subscriber('/barlas/encoders/raw', Float32MultiArray, self.encoder_callback)
        self.odom_sub = rospy.Subscriber('/barlas/odom', Odometry, self.odometry_callback)
        
        # Encoder durumu
        self.current_encoders = {'left': 0, 'right': 0}
        self.current_odom = None
        
        # === Obstacle Avoidance ===
        self.obstacle_avoidance_pub = rospy.Publisher('/barlas/navigation/cmd_vel', Twist, queue_size=1)
        
        # === System State ===
        self.system_status_pub = rospy.Publisher('/barlas/system/status', String, queue_size=1)
        
        # Durumlar
        self.dart_targeting_active = False
        self.obstacle_avoidance_active = True
        self.current_dart_position = None
        self.sensor_data = {}
        
        # Timer'lar
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.main_control_loop)  # 10Hz
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_system_status)  # 1Hz
        
        print("✅ [BARLAS Bridge] Sistem hazır - tüm bağlantılar aktif!")
        
    def mavros_state_callback(self, msg):
        """MAVROS bağlantı durumu"""
        self.mavros_state = msg
        self.mavros_armed = msg.armed
        self.current_mode = msg.mode
        
    def dart_detected_callback(self, msg):
        """Dart tespit edildiğinde"""
        self.current_dart_position = msg.point
        rospy.loginfo(f"🎯 [Dart] Hedef tespit edildi: ({msg.point.x:.1f}, {msg.point.y:.1f})")
        
        # Otomatik hedefleme
        if self.dart_targeting_active:
            self.engage_dart_target()
    
    def laser_status_callback(self, msg):
        """Lazer durumu güncelleme"""
        status = "AÇIK" if msg.data else "KAPALI"
        rospy.loginfo(f"🔴 [Laser] Durum: {status}")
    
    def sensor_data_callback(self, msg):
        """Raspberry Pi'dan gelen sensör verileri"""
        try:
            self.sensor_data = json.loads(msg.data)
            # Engel algılama mantığı burada
            if self.obstacle_avoidance_active:
                self.process_obstacle_avoidance()
        except Exception as e:
            rospy.logwarn(f"Sensör veri çözümleme hatası: {e}")
    
    def encoder_callback(self, msg):
        """Arduino encoder verileri callback"""
        try:
            if len(msg.data) >= 2:
                self.current_encoders['left'] = msg.data[0]
                self.current_encoders['right'] = msg.data[1]
                
                # Debug (her 5 saniyede bir)
                if int(rospy.Time.now().to_sec()) % 5 == 0:
                    rospy.logdebug(f"📊 [Encoder] Sol:{self.current_encoders['left']:.0f}, Sağ:{self.current_encoders['right']:.0f}")
        except Exception as e:
            rospy.logwarn(f"Encoder veri çözümleme hatası: {e}")
    
    def odometry_callback(self, msg):
        """Robot odometry callback"""
        try:
            self.current_odom = msg
            
            # Pozisyon bilgilerini al
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Yaw açısını quaternion'dan çıkar
            import tf2_ros
            from tf2_geometry_msgs import do_transform_pose
            # Basit euler açısı hesapla
            orientation = msg.pose.pose.orientation
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Debug (her 2 saniyede bir)
            if int(rospy.Time.now().to_sec()) % 2 == 0:
                rospy.logdebug(f"🗺️ [Odom] Pos:({x:.2f},{y:.2f}) Yaw:{math.degrees(yaw):.1f}°")
                
        except Exception as e:
            rospy.logwarn(f"Odometry veri çözümleme hatası: {e}")
    
    def ultrasonic_callback(self, msg):
        """8x ultrasonik sensör verisi"""
        # En yakın engel mesafesi
        min_distance = min(msg.ranges)
        if min_distance < 1.0:  # 1 metre yakın engel
            rospy.logwarn(f"⚠️  [Engel] Yakın engel: {min_distance:.2f}m")
            if self.obstacle_avoidance_active:
                self.avoid_obstacle(msg)
    
    def imu_callback(self, msg):
        """IMU verisi - eğim ve yönelim"""
        # Eğim kontrolü
        # orientation quaternion'dan eğim çıkar
        pass
    
    def main_control_loop(self, event):
        """Ana kontrol döngüsü - 10Hz"""
        if not self.mavros_state or not self.mavros_state.connected:
            return
            
        # Dart hedefleme modu kontrolü
        if self.dart_targeting_active and self.current_dart_position:
            self.track_dart_target()
        
        # Mission Planner ile haberleşme
        self.update_mission_planner_commands()
    
    def engage_dart_target(self):
        """Dart hedefini kilitle ve takip et"""
        if not self.current_dart_position:
            return
            
        rospy.loginfo("🎯 [Dart] Hedef kilitlendi - pan-tilt harekete geçiyor")
        
        # Pan-tilt servo'yu dart pozisyonuna yönlendir
        servo_cmd = Float32MultiArray()
        # Kamera koordinatından servo açısına dönüşüm
        pan_angle = self.pixel_to_servo_angle(self.current_dart_position.x, 'pan')
        tilt_angle = self.pixel_to_servo_angle(self.current_dart_position.y, 'tilt')
        
        servo_cmd.data = [pan_angle, tilt_angle]
        self.servo_position_pub.publish(servo_cmd)
        
        # 2 saniye sonra lazeri aç
        rospy.Timer(rospy.Duration(2.0), self.fire_laser_callback, oneshot=True)
    
    def fire_laser_callback(self, event):
        """Lazer ateşleme"""
        rospy.loginfo("🔥 [Laser] ATEŞ!")
        laser_cmd = Bool()
        laser_cmd.data = True
        self.servo_laser_pub.publish(laser_cmd)
        
        # 1 saniye sonra lazeri kapat
        rospy.Timer(rospy.Duration(1.0), self.turn_off_laser_callback, oneshot=True)
    
    def turn_off_laser_callback(self, event):
        """Lazer kapatma"""
        laser_cmd = Bool()
        laser_cmd.data = False
        self.servo_laser_pub.publish(laser_cmd)
        rospy.loginfo("🔴 [Laser] Kapatıldı")
        
        # Dart hedefleme tamamlandı
        self.dart_targeting_active = False
        self.current_dart_position = None
    
    def process_obstacle_avoidance(self):
        """Engelden kaçınma algoritması"""
        if 'ultrasonic' in self.sensor_data:
            distances = self.sensor_data['ultrasonic']
            
            # A* algoritması burada implementt edilecek
            # Şimdilik basit logic
            min_dist = min(distances.values())
            if min_dist < 2.0:  # 2 metre içinde engel
                # Geri çekil veya yön değiştir
                avoid_cmd = Twist()
                avoid_cmd.linear.x = -0.2  # Geri
                avoid_cmd.angular.z = 0.5  # Sola dön
                self.obstacle_avoidance_pub.publish(avoid_cmd)
                rospy.loginfo("🚧 [Engel] Kaçınma manevras
    
    def avoid_obstacle(self, laser_scan):
        """LaserScan verisinden engel kaçınma"""
        # Basit engel kaçınma
        front_distances = laser_scan.ranges[len(laser_scan.ranges)//4:-len(laser_scan.ranges)//4]
        min_front_dist = min(front_distances)
        
        if min_front_dist < 1.5:
            # Stop and turn
            rc_override = OverrideRCIn()
            rc_override.channels = [1500, 1500, 1000, 1700, 0, 0, 0, 0]  # Stop + turn right
            self.rc_override_pub.publish(rc_override)
    
    def update_mission_planner_commands(self):
        """Mission Planner'a RC override komutları gönder"""
        if not self.obstacle_avoidance_active:
            return
            
        # Normal sürüş komutu
        rc_override = OverrideRCIn()
        rc_override.channels = [1500, 1600, 1000, 1500, 0, 0, 0, 0]  # İleri hareket
        self.rc_override_pub.publish(rc_override)
    
    def pixel_to_servo_angle(self, pixel_coord, axis):
        """Piksel koordinatından servo açısına dönüşüm"""
        if axis == 'pan':
            # Kamera genişliği 640 pixel, servo 0-180 derece
            angle = (pixel_coord / 640.0) * 180.0
        else:  # tilt
            # Kamera yüksekliği 480 pixel, servo 0-180 derece
            angle = (pixel_coord / 480.0) * 180.0
        return max(0, min(180, angle))
    
    def publish_system_status(self, event):
        """Sistem durumu yayınla"""
        status = {
            'mavros_connected': self.mavros_state.connected if self.mavros_state else False,
            'armed': self.mavros_armed,
            'mode': self.current_mode,
            'dart_targeting': self.dart_targeting_active,
            'obstacle_avoidance': self.obstacle_avoidance_active,
            'encoders': self.current_encoders,
            'odom_available': self.current_odom is not None
        }
        
        # Odometry bilgisi varsa ekle
        if self.current_odom:
            status['position'] = {
                'x': self.current_odom.pose.pose.position.x,
                'y': self.current_odom.pose.pose.position.y,
                'z': self.current_odom.pose.pose.position.z
            }
            status['velocity'] = {
                'linear_x': self.current_odom.twist.twist.linear.x,
                'angular_z': self.current_odom.twist.twist.angular.z
            }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.system_status_pub.publish(status_msg)
    
    # === Public API Functions ===
    
    def enable_dart_targeting(self):
        """Dart hedefleme sistemini aktif et"""
        self.dart_targeting_active = True
        targeting_msg = Bool()
        targeting_msg.data = True
        self.targeting_enable_pub.publish(targeting_msg)
        rospy.loginfo("🎯 [System] Dart hedefleme sistemi AKTİF")
    
    def disable_dart_targeting(self):
        """Dart hedefleme sistemini pasif et"""
        self.dart_targeting_active = False
        targeting_msg = Bool()
        targeting_msg.data = False
        self.targeting_enable_pub.publish(targeting_msg)
        rospy.loginfo("🎯 [System] Dart hedefleme sistemi PASİF")
    
    def enable_obstacle_avoidance(self):
        """Engel kaçınmayı aktif et"""
        self.obstacle_avoidance_active = True
        rospy.loginfo("🚧 [System] Engel kaçınma AKTİF")
    
    def disable_obstacle_avoidance(self):
        """Engel kaçınmayı pasif et"""
        self.obstacle_avoidance_active = False
        rospy.loginfo("🚧 [System] Engel kaçınma PASİF")

def main():
    try:
        bridge = BARLASROSBridge()
        
        print("🎯 BARLAS Hibrit Sistem Başlatıldı!")
        print("✅ WSL2 ROS Noetic")
        print("✅ Windows Mission Planner")  
        print("✅ Dart Laser System")
        print("✅ Obstacle Avoidance")
        print("✅ Raspberry Pi Sensors")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("🛑 [BARLAS Bridge] Sistem kapatılıyor...")
    except Exception as e:
        print(f"❌ [BARLAS Bridge] HATA: {e}")

if __name__ == '__main__':
    main()
