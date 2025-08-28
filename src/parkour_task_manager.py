#!/usr/bin/env python3
"""
BARLAS Parkour Task Manager
Teknofest İnsansız Kara Aracı Yarışması - Spesifik Parkur Görevleri
Her parkur etabı için özelleştirilmiş kontrol algoritmaları
"""

import rospy
import time
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan
from typing import Dict, List, Optional, Tuple

class ParkourTaskManager:
    """
    Teknofest parkur görevlerini yöneten sınıf
    - Dik eğim (45° tırmanma)
    - Yan eğim (20% yanal eğim)
    - Sığ su (40cm derinlik)
    - Çakıllı yol
    - Trafik konileri (slalom)
    - Hızlanma bölgesi
    """
    
    def __init__(self):
        rospy.init_node('barlas_parkour_manager', anonymous=True)
        
        # Parkour görev durumu
        self.current_task = None
        self.task_start_time = None
        self.task_completed = False
        
        # Sensör verileri
        self.imu_data = None
        self.lidar_data = None
        self.ultrasonic_ranges = {}
        self.gps_position = None
        
        # Kontrol komutları
        self.cmd_vel_pub = rospy.Publisher('/barlas/control/cmd_vel', Twist, queue_size=1)
        self.task_status_pub = rospy.Publisher('/barlas/parkour/status', String, queue_size=1)
        
        # Sensör subscribers
        self.imu_sub = rospy.Subscriber('/barlas/sensors/imu', String, self.imu_callback)
        self.lidar_sub = rospy.Subscriber('/barlas/sensors/scan', LaserScan, self.lidar_callback)
        self.ultrasonic_sub = rospy.Subscriber('/barlas/sensors/ultrasonic/front', Range, self.ultrasonic_callback)
        
        # Parkour parametreleri
        self.parkour_params = self._initialize_parkour_parameters()
        
        rospy.loginfo("🏁 BARLAS Parkour Task Manager başlatıldı!")
    
    def _initialize_parkour_parameters(self) -> Dict:
        """Parkur parametrelerini initialize et"""
        return {
            "dik_eğim": {
                "max_speed": 0.3,
                "max_angle": 45.0,
                "approach_distance": 2.0,
                "climb_duration": 60.0,
                "brake_threshold": 40.0
            },
            "yan_eğim": {
                "max_speed": 0.4,
                "max_tilt": 20.0,
                "correction_gain": 0.8,
                "traverse_duration": 45.0
            },
            "sığ_su": {
                "max_speed": 0.25,
                "water_depth": 0.4,  # 40cm
                "crossing_duration": 90.0,
                "steady_mode": True
            },
            "çakıllı_yol": {
                "max_speed": 0.4,
                "vibration_threshold": 2.0,
                "traction_mode": "enhanced",
                "duration": 30.0
            },
            "trafik_konileri": {
                "max_speed": 0.35,
                "cone_distance": 3.0,
                "slalom_width": 2.0,
                "precision_mode": True,
                "duration": 120.0
            },
            "hızlanma": {
                "max_speed": 1.0,
                "acceleration": 0.5,
                "duration": 20.0,
                "brake_disabled": True
            }
        }
    
    def imu_callback(self, msg):
        """IMU sensor verisi"""
        # Mock IMU data processing
        self.imu_data = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'timestamp': time.time()
        }
    
    def lidar_callback(self, msg):
        """LiDAR verisi"""
        self.lidar_data = {
            'ranges': msg.ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'timestamp': time.time()
        }
    
    def ultrasonic_callback(self, msg):
        """Ultrasonik sensör verisi"""
        self.ultrasonic_ranges['front'] = msg.range
    
    def start_parkour_task(self, task_name: str) -> bool:
        """Parkur görevini başlat"""
        if task_name not in self.parkour_params:
            rospy.logerr(f"❌ Bilinmeyen parkur görevi: {task_name}")
            return False
        
        rospy.loginfo(f"🏃 Parkur görevi başlatılıyor: {task_name}")
        
        self.current_task = task_name
        self.task_start_time = time.time()
        self.task_completed = False
        
        # Görev-spesifik thread başlat
        task_thread = threading.Thread(target=self._execute_parkour_task)
        task_thread.daemon = True
        task_thread.start()
        
        return True
    
    def _execute_parkour_task(self):
        """Parkur görevini execute et"""
        if not self.current_task:
            return
        
        rospy.loginfo(f"🔄 {self.current_task} görevi başladı")
        
        try:
            if self.current_task == "dik_eğim":
                self._handle_steep_climb()
            elif self.current_task == "yan_eğim":
                self._handle_side_slope()
            elif self.current_task == "sığ_su":
                self._handle_water_crossing()
            elif self.current_task == "çakıllı_yol":
                self._handle_gravel_road()
            elif self.current_task == "trafik_konileri":
                self._handle_traffic_cones()
            elif self.current_task == "hızlanma":
                self._handle_acceleration_zone()
            
            # Görev tamamlandı
            self.task_completed = True
            completion_time = time.time() - self.task_start_time
            rospy.loginfo(f"✅ {self.current_task} görevi tamamlandı! Süre: {completion_time:.1f}s")
            
        except Exception as e:
            rospy.logerr(f"❌ Parkur görevi hatası ({self.current_task}): {e}")
        finally:
            self._stop_vehicle()
    
    def _handle_steep_climb(self):
        """Dik eğim tırmanma (45°)"""
        params = self.parkour_params["dik_eğim"]
        rospy.loginfo("⛰️ Dik eğim tırmanma başlıyor...")
        
        # Faz 1: Eğime yaklaşım
        rospy.loginfo("📍 Faz 1: Eğime yaklaşım")
        self._approach_obstacle(params["approach_distance"])
        
        # Faz 2: Eğim tırmanma
        rospy.loginfo("📈 Faz 2: Eğim tırmanma")
        climb_start = time.time()
        
        while (time.time() - climb_start) < params["climb_duration"]:
            # IMU ile eğim açısı kontrolü
            if self.imu_data:
                pitch_angle = abs(self.imu_data.get('pitch', 0))
                
                if pitch_angle > params["brake_threshold"]:
                    rospy.logwarn(f"⚠️ Kritik eğim açısı: {pitch_angle:.1f}°")
                    # Freni aktif et, daha yavaş ilerle
                    cmd = Twist()
                    cmd.linear.x = 0.1  # Çok yavaş
                    self.cmd_vel_pub.publish(cmd)
                else:
                    # Normal eğim tırmanma hızı
                    cmd = Twist()
                    cmd.linear.x = params["max_speed"]
                    self.cmd_vel_pub.publish(cmd)
            
            # Engel kontrolü
            if self._check_obstacle_ahead(1.0):  # 1m ahead check
                rospy.loginfo("🚧 Engel algılandı - yavaşlıyor")
                time.sleep(1)
            
            time.sleep(0.1)  # 10Hz kontrol
        
        # Faz 3: Eğim zirvesi
        rospy.loginfo("🏔️ Faz 3: Eğim zirvesi geçildi")
    
    def _handle_side_slope(self):
        """Yan eğim (%20 yanal eğim)"""
        params = self.parkour_params["yan_eğim"]
        rospy.loginfo("🔄 Yan eğim geçişi başlıyor...")
        
        traverse_start = time.time()
        
        while (time.time() - traverse_start) < params["traverse_duration"]:
            if self.imu_data:
                roll_angle = self.imu_data.get('roll', 0)
                
                # Yan eğim kompanzasyonu
                correction = -roll_angle * params["correction_gain"]
                
                cmd = Twist()
                cmd.linear.x = params["max_speed"]
                cmd.angular.z = correction  # Yatılma düzeltme
                
                self.cmd_vel_pub.publish(cmd)
                
                if abs(roll_angle) > params["max_tilt"]:
                    rospy.logwarn(f"⚠️ Kritik yan eğim: {roll_angle:.1f}°")
            
            time.sleep(0.1)
    
    def _handle_water_crossing(self):
        """Sığ su geçişi (40cm derinlik)"""
        params = self.parkour_params["sığ_su"]
        rospy.loginfo("💧 Su geçişi başlıyor...")
        
        # Su tespiti (sensör nem oranı ile)
        water_detected = self._detect_water()
        
        if not water_detected:
            rospy.logwarn("⚠️ Su algılanmadı - normal ilerle")
        
        crossing_start = time.time()
        
        while (time.time() - crossing_start) < params["crossing_duration"]:
            # Yavaş ve kararlı hareket
            cmd = Twist()
            cmd.linear.x = params["max_speed"]
            cmd.angular.z = 0.0  # Düz ilerle
            
            self.cmd_vel_pub.publish(cmd)
            
            # Su seviyesi kontrolü
            water_level = self._get_water_level()
            if water_level > params["water_depth"]:
                rospy.logwarn(f"🌊 Yüksek su seviyesi: {water_level:.2f}m")
                # Daha yavaş ilerle
                cmd.linear.x = 0.1
                self.cmd_vel_pub.publish(cmd)
            
            time.sleep(0.1)
        
        rospy.loginfo("🏊 Su geçişi tamamlandı!")
    
    def _handle_gravel_road(self):
        """Çakıllı yol"""
        params = self.parkour_params["çakıllı_yol"]
        rospy.loginfo("🪨 Çakıllı yol geçişi başlıyor...")
        
        gravel_start = time.time()
        
        while (time.time() - gravel_start) < params["duration"]:
            # Titreşim seviyesi kontrolü (IMU ivme verisi)
            vibration_level = self._get_vibration_level()
            
            if vibration_level > params["vibration_threshold"]:
                # Yüksek titreşim - hızı azalt
                speed = params["max_speed"] * 0.7
            else:
                speed = params["max_speed"]
            
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_vel_pub.publish(cmd)
            
            time.sleep(0.1)
    
    def _handle_traffic_cones(self):
        """Trafik konileri (slalom)"""
        params = self.parkour_params["trafik_konileri"]
        rospy.loginfo("🚧 Trafik konileri slalom başlıyor...")
        
        slalom_start = time.time()
        cone_side = "left"  # İlk koni hangi tarafta
        
        while (time.time() - slalom_start) < params["duration"]:
            # LiDAR ile koni tespiti
            cones = self._detect_traffic_cones()
            
            if cones:
                closest_cone = min(cones, key=lambda c: c['distance'])
                
                if closest_cone['distance'] < params["cone_distance"]:
                    # Koni yakın - slalom manevrası
                    angular_velocity = 0.8 if cone_side == "left" else -0.8
                    
                    cmd = Twist()
                    cmd.linear.x = params["max_speed"]
                    cmd.angular.z = angular_velocity
                    
                    self.cmd_vel_pub.publish(cmd)
                    
                    # Koni tarafını değiştir
                    cone_side = "right" if cone_side == "left" else "left"
                    
                    time.sleep(2)  # Manevraya devam
                else:
                    # Düz ilerle
                    cmd = Twist()
                    cmd.linear.x = params["max_speed"]
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)
            else:
                # Koni bulunamadı - ara
                cmd = Twist()
                cmd.linear.x = 0.2
                cmd.angular.z = 0.3  # Hafif dönüş
                self.cmd_vel_pub.publish(cmd)
            
            time.sleep(0.1)
    
    def _handle_acceleration_zone(self):
        """Hızlanma bölgesi"""
        params = self.parkour_params["hızlanma"]
        rospy.loginfo("🚀 Hızlanma bölgesi başlıyor...")
        
        accel_start = time.time()
        current_speed = 0.0
        
        while (time.time() - accel_start) < params["duration"]:
            # Kademeli hızlanma
            elapsed = time.time() - accel_start
            target_speed = min(params["max_speed"], current_speed + params["acceleration"] * elapsed)
            
            cmd = Twist()
            cmd.linear.x = target_speed
            self.cmd_vel_pub.publish(cmd)
            
            current_speed = target_speed
            time.sleep(0.1)
        
        rospy.loginfo(f"💨 Maksimum hız ulaşıldı: {current_speed:.2f} m/s")
    
    # === Yardımcı Fonksiyonlar ===
    
    def _approach_obstacle(self, distance: float):
        """Engele yaklaşım"""
        while self._get_front_distance() > distance:
            cmd = Twist()
            cmd.linear.x = 0.3  # Yaklaşım hızı
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
    
    def _check_obstacle_ahead(self, threshold: float) -> bool:
        """Önde engel kontrolü"""
        front_distance = self._get_front_distance()
        return front_distance < threshold
    
    def _get_front_distance(self) -> float:
        """Ön mesafe al"""
        if 'front' in self.ultrasonic_ranges:
            return self.ultrasonic_ranges['front']
        elif self.lidar_data:
            # LiDAR front sector average
            ranges = self.lidar_data['ranges']
            front_sector = ranges[len(ranges)//2-10:len(ranges)//2+10]
            return min(front_sector) if front_sector else float('inf')
        return float('inf')
    
    def _detect_water(self) -> bool:
        """Su tespiti (nem sensörü ile)"""
        # Mock water detection
        return True
    
    def _get_water_level(self) -> float:
        """Su seviyesi al"""
        # Mock water level (0-0.5m)
        return 0.2
    
    def _get_vibration_level(self) -> float:
        """Titreşim seviyesi al"""
        # Mock vibration from IMU
        return 1.0
    
    def _detect_traffic_cones(self) -> List[Dict]:
        """Trafik koni tespiti (LiDAR ile)"""
        # Mock cone detection
        return [
            {'distance': 2.5, 'angle': 0.2, 'side': 'left'},
            {'distance': 4.0, 'angle': -0.3, 'side': 'right'}
        ]
    
    def _stop_vehicle(self):
        """Aracı durdur"""
        cmd = Twist()  # Tüm sıfır
        self.cmd_vel_pub.publish(cmd)
    
    def get_task_status(self) -> Dict:
        """Görev durumunu döndür"""
        return {
            'current_task': self.current_task,
            'completed': self.task_completed,
            'elapsed_time': time.time() - self.task_start_time if self.task_start_time else 0,
            'sensor_status': {
                'imu_active': self.imu_data is not None,
                'lidar_active': self.lidar_data is not None,
                'ultrasonic_active': len(self.ultrasonic_ranges) > 0
            }
        }

def main():
    """Test fonksiyonu"""
    try:
        parkour_manager = ParkourTaskManager()
        
        rospy.loginfo("🎮 Parkour Task Manager Test Modu")
        rospy.loginfo("Örnek görevler: dik_eğim, yan_eğim, sığ_su, çakıllı_yol, trafik_konileri, hızlanma")
        
        # Test görevi başlat
        test_task = "dik_eğim"  # Varsayılan test
        parkour_manager.start_parkour_task(test_task)
        
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("⏹️ Parkour Manager durduruldu")
    except Exception as e:
        rospy.logerr(f"❌ Parkour Manager hatası: {e}")

if __name__ == "__main__":
    main()
