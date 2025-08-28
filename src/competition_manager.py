#!/usr/bin/env python3
"""
BARLAS Competition Manager
Teknofest İnsansız Kara Aracı Yarışması - Ana Yarışma Kontrol Sistemi
Tüm parkur görevlerini koordine eden merkezi sistem
"""

import rospy
import time
import threading
import json
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from typing import Dict, List, Optional

# BARLAS modülleri
from traffic_sign_recognition import TrafficSignRecognizer
from dart_laser_system.dart_detector import DartDetector
from obstacle_avoidance import ObstacleAvoidance
from pid_controller import PIDController

class BARLASCompetitionManager:
    """
    Teknofest yarışma yöneticisi
    - Parkur görevlerini takip eder
    - Puanlama sistemi
    - Zaman yönetimi
    - Güvenlik kontrolü
    - Mission planner entegrasyonu
    """
    
    def __init__(self):
        rospy.init_node('barlas_competition_manager', anonymous=True)
        
        # Yarışma durumu
        self.competition_state = "STANDBY"  # STANDBY, RUNNING, DART_ZONE, COMPLETED, EMERGENCY
        self.current_task = "başlangıç"
        self.total_score = 0
        self.start_time = None
        self.task_completion_times = {}
        
        # Parkur görevleri
        self.competition_tasks = {
            "otonom_sürüş": {"completed": False, "score": 0, "max_score": 20},
            "dik_eğim": {"completed": False, "score": 0, "max_score": 15},
            "yan_eğim": {"completed": False, "score": 0, "max_score": 10},
            "sığ_su": {"completed": False, "score": 0, "max_score": 15},
            "çakıllı_yol": {"completed": False, "score": 0, "max_score": 10},
            "trafik_konileri": {"completed": False, "score": 0, "max_score": 20},
            "dart_atış": {"completed": False, "score": 0, "max_score": 30},
            "tabela_algılama": {"completed": False, "score": 0, "max_score": 15}
        }
        
        # Alt sistemler
        self.traffic_sign_recognizer = TrafficSignRecognizer()
        self.dart_detector = DartDetector(confidence_threshold=0.6)
        self.obstacle_avoidance = ObstacleAvoidance()
        self.pid_controller = PIDController()
        
        # ROS Publishers/Subscribers
        self.mission_status_pub = rospy.Publisher('/barlas/competition/status', String, queue_size=1)
        self.score_pub = rospy.Publisher('/barlas/competition/score', Float32, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/barlas/control/cmd_vel', Twist, queue_size=1)
        
        # GPS tracking
        self.gps_sub = rospy.Subscriber('/barlas/sensors/gps', NavSatFix, self.gps_callback)
        self.current_position = None
        self.start_position = None
        self.waypoints = []
        
        # Kamera feed
        self.camera_active = False
        
        # Güvenlik
        self.emergency_stop = False
        self.safety_violations = 0
        self.max_violations = 3
        
        # Timer'lar
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        self.safety_timer = rospy.Timer(rospy.Duration(0.5), self.safety_check)
        
        rospy.loginfo("🏁 BARLAS Competition Manager başlatıldı!")
    
    def gps_callback(self, msg):
        """GPS pozisyon güncelle"""
        self.current_position = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }
        
        if self.start_position is None and self.competition_state == "RUNNING":
            self.start_position = self.current_position.copy()
    
    def start_competition(self):
        """Yarışmayı başlat"""
        if self.competition_state != "STANDBY":
            rospy.logwarn("⚠️ Yarışma zaten başlamış!")
            return False
        
        rospy.loginfo("🚀 TEKNOFEST YARIŞMASI BAŞLIYOR!")
        
        self.competition_state = "RUNNING"
        self.start_time = time.time()
        self.current_task = "otonom_sürüş"
        
        # Competition thread başlat
        self.competition_thread = threading.Thread(target=self.competition_main_loop)
        self.competition_thread.daemon = True
        self.competition_thread.start()
        
        return True
    
    def competition_main_loop(self):
        """Ana yarışma döngüsü"""
        rospy.loginfo("🔄 Yarışma ana döngüsü başladı")
        
        try:
            while not rospy.is_shutdown() and self.competition_state == "RUNNING":
                
                # Mevcut görevi işle
                if self.current_task == "otonom_sürüş":
                    self.handle_autonomous_driving()
                
                elif self.current_task == "tabela_algılama":
                    self.handle_traffic_sign_detection()
                
                elif self.current_task == "dart_zone":
                    self.competition_state = "DART_ZONE"
                    self.handle_dart_targeting()
                
                elif self.current_task == "engel_kaçınma":
                    self.handle_obstacle_avoidance()
                
                # Görev tamamlanma kontrolü
                self.check_task_completion()
                
                # Yarışma süresi kontrolü (maksimum 15 dakika)
                if time.time() - self.start_time > 900:  # 15 dakika
                    rospy.logwarn("⏰ Yarışma süresi doldu!")
                    self.end_competition("TIME_UP")
                    break
                
                time.sleep(0.1)  # 10Hz döngü
                
        except Exception as e:
            rospy.logerr(f"❌ Yarışma döngüsü hatası: {e}")
            self.emergency_stop_procedure()
    
    def handle_autonomous_driving(self):
        """Otonom sürüş görevi"""
        # Engellerden kaçınarak ilerle
        linear_vel, angular_vel = self.obstacle_avoidance.calculate_avoidance_vector()
        
        # PID kontrol ile hız ayarlama
        adjusted_linear = self.pid_controller.compute(linear_vel)
        
        # Hareket komutu gönder
        cmd = Twist()
        cmd.linear.x = adjusted_linear
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        # İlerleme skorlama (GPS bazlı)
        if self.current_position and self.start_position:
            distance_covered = self.calculate_distance_covered()
            
            # 50 metre ilerlemede puan ver
            if distance_covered >= 50.0:
                self.award_points("otonom_sürüş", min(20, int(distance_covered / 10)))
    
    def handle_traffic_sign_detection(self):
        """Tabela algılama görevi"""
        # Kameradan frame al (simüle edilmiş)
        # Gerçek uygulamada cv_bridge ile ROS Image alınacak
        
        try:
            # Tabela tespiti (mock data)
            detected_signs = []  # self.traffic_sign_recognizer.detect_traffic_signs(frame)
            
            for sign in detected_signs:
                sign_class = sign['class']
                confidence = sign['confidence']
                
                if confidence >= 0.7:
                    rospy.loginfo(f"🚦 Tabela algılandı: {sign_class}")
                    
                    # Mission komutu al
                    mission_cmd = self.traffic_sign_recognizer.get_mission_command_from_sign(sign_class)
                    
                    # PID parametrelerini güncelle
                    if 'pid_gains' in mission_cmd:
                        self.pid_controller.set_gains(
                            mission_cmd['pid_gains']['kp'],
                            mission_cmd['pid_gains']['ki'],
                            mission_cmd['pid_gains']['kd']
                        )
                    
                    # Tabela puanı ver
                    self.award_points("tabela_algılama", 5)
                    
                    # Parkur görevine geç
                    self.transition_to_parkour_task(sign_class)
                    
        except Exception as e:
            rospy.logwarn(f"Tabela algılama hatası: {e}")
    
    def handle_dart_targeting(self):
        """Dart hedefleme görevi"""
        rospy.loginfo("🎯 Dart hedefleme modu aktif")
        
        dart_task_start = time.time()
        dart_attempts = 0
        max_attempts = 3
        
        while dart_attempts < max_attempts and time.time() - dart_task_start < 120:  # 2 dakika limit
            
            # Dart tespiti (mock)
            # dart_detections = self.dart_detector.detect_darts(frame)
            
            # Hedef bulundu mu?
            target_acquired = False  # len(dart_detections) > 0
            
            if target_acquired:
                rospy.loginfo("🎯 Dart hedefi kilitlendi!")
                
                # Pan-tilt hedefleme (2 saniye)
                time.sleep(2)
                
                # Lazer atış simülasyonu
                rospy.loginfo("🔴 Lazer ateşlendi!")
                
                # Başarı skoru (mesafeye göre)
                hit_score = 15  # Merkeze yakınlığa göre 5-15 puan
                self.award_points("dart_atış", hit_score)
                
                dart_attempts += 1
                time.sleep(3)  # Tekrar deneme arası
            
            else:
                # Dart arama (360° dönüş)
                cmd = Twist()
                cmd.angular.z = 0.5  # Yavaş dönüş
                self.cmd_vel_pub.publish(cmd)
                time.sleep(1)
        
        # Dart görevi tamamlandı
        self.complete_task("dart_atış")
        self.competition_state = "RUNNING"
        self.current_task = "otonom_sürüş"  # Ana görev devam
    
    def handle_obstacle_avoidance(self):
        """Engel kaçınma görevi"""
        avoidance_result = self.obstacle_avoidance.get_obstacle_status()
        
        if avoidance_result.get('obstacles_detected'):
            rospy.loginfo("🚧 Engel algılandı - kaçınma maneveri")
            
            # Kaçınma puanı ver
            self.award_points("otonom_sürüş", 2)
    
    def transition_to_parkour_task(self, sign_class):
        """Tabela algılamasından parkur görevine geçiş"""
        task_mapping = {
            "dik_eğim": "dik_eğim",
            "yan_eğim": "yan_eğim", 
            "sığ_su": "sığ_su",
            "çakıllı_yol": "çakıllı_yol",
            "trafik_konileri": "trafik_konileri",
            "dur": "dur_bekle"
        }
        
        new_task = task_mapping.get(sign_class)
        if new_task:
            rospy.loginfo(f"📋 Görev değişimi: {self.current_task} -> {new_task}")
            self.current_task = new_task
            
            # Görev başlama zamanını kaydet
            self.task_completion_times[new_task] = {"start": time.time()}
    
    def check_task_completion(self):
        """Görev tamamlanma kontrolü"""
        
        # Parkur görevleri için mesafe/süre bazlı kontrol
        if self.current_task in self.competition_tasks:
            task_start_time = self.task_completion_times.get(self.current_task, {}).get("start")
            
            if task_start_time:
                elapsed_time = time.time() - task_start_time
                
                # Görev süresi kontrolü (her görev için farklı)
                task_duration_limits = {
                    "dik_eğim": 60,      # 1 dakika
                    "yan_eğim": 45,      # 45 saniye
                    "sığ_su": 90,        # 1.5 dakika
                    "çakıllı_yol": 30,   # 30 saniye
                    "trafik_konileri": 120  # 2 dakika
                }
                
                duration_limit = task_duration_limits.get(self.current_task, 60)
                
                if elapsed_time >= duration_limit:
                    rospy.loginfo(f"✅ Görev tamamlandı: {self.current_task}")
                    self.complete_task(self.current_task)
    
    def complete_task(self, task_name):
        """Görevi tamamla ve puan ver"""
        if task_name in self.competition_tasks:
            task_info = self.competition_tasks[task_name]
            
            if not task_info["completed"]:
                task_info["completed"] = True
                
                # Süre bonusu hesapla
                task_start = self.task_completion_times.get(task_name, {}).get("start")
                if task_start:
                    completion_time = time.time() - task_start
                    self.task_completion_times[task_name]["completion"] = completion_time
                    
                    # Hızlı tamamlama bonusu
                    time_bonus = max(0, 10 - int(completion_time / 10))
                    total_score = task_info["max_score"] + time_bonus
                    
                    self.award_points(task_name, total_score)
                    
                rospy.loginfo(f"🏆 {task_name} görevi tamamlandı! Puan: {task_info['score']}")
    
    def award_points(self, task_name, points):
        """Puan ver"""
        if task_name in self.competition_tasks:
            self.competition_tasks[task_name]["score"] += points
            self.total_score += points
            
            rospy.loginfo(f"📊 Puan: +{points} ({task_name}) | Toplam: {self.total_score}")
            
            # Score publish et
            self.score_pub.publish(Float32(data=self.total_score))
    
    def safety_check(self, event):
        """Güvenlik kontrolü"""
        if self.emergency_stop:
            return
            
        # Kritik sensör kontrolleri
        # - Batarya seviyesi
        # - IMU eğim açısı  
        # - LiDAR yakın engel
        # - GPS sinyal kalitesi
        
        violations = []
        
        # Mock safety checks
        battery_level = 85  # self.get_battery_level()
        if battery_level < 20:
            violations.append("DÜŞÜK_BATARYA")
        
        imu_tilt = 15  # self.get_imu_tilt()
        if imu_tilt > 45:
            violations.append("AŞIRI_EĞİM") 
            
        if violations:
            self.safety_violations += len(violations)
            rospy.logwarn(f"⚠️ Güvenlik uyarısı: {violations}")
            
            if self.safety_violations >= self.max_violations:
                self.emergency_stop_procedure()
    
    def emergency_stop_procedure(self):
        """Acil durdurma prosedürü"""
        rospy.logerr("🚨 ACİL DURDURMA PROSEDÜRÜ AKTİF!")
        
        self.emergency_stop = True
        self.competition_state = "EMERGENCY"
        
        # Tüm hareketleri durdur
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Fren sistemi aktif et (motor driver'a komut)
        # self.motor_driver.apply_fren()
        
        self.end_competition("EMERGENCY_STOP")
    
    def end_competition(self, reason="COMPLETED"):
        """Yarışmayı sonlandır"""
        end_time = time.time()
        total_duration = end_time - self.start_time if self.start_time else 0
        
        rospy.loginfo(f"🏁 YARIŞMA SONLANDI: {reason}")
        rospy.loginfo(f"⏱️ Süre: {total_duration:.1f} saniye")
        rospy.loginfo(f"📊 Toplam Puan: {self.total_score}")
        
        # Final rapor
        self.generate_competition_report()
        
        self.competition_state = "COMPLETED"
    
    def generate_competition_report(self):
        """Yarışma raporu oluştur"""
        report = {
            "yarışma_süresi": time.time() - self.start_time if self.start_time else 0,
            "toplam_puan": self.total_score,
            "tamamlanan_görevler": [],
            "görev_süreleri": self.task_completion_times
        }
        
        for task_name, task_info in self.competition_tasks.items():
            if task_info["completed"]:
                report["tamamlanan_görevler"].append({
                    "görev": task_name,
                    "puan": task_info["score"],
                    "maksimum_puan": task_info["max_score"]
                })
        
        # JSON raporu kaydet
        report_filename = f"/tmp/barlas_competition_report_{int(time.time())}.json"
        with open(report_filename, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        
        rospy.loginfo(f"📄 Yarışma raporu kaydedildi: {report_filename}")
    
    def calculate_distance_covered(self):
        """GPS ile kat edilen mesafeyi hesapla"""
        if not self.start_position or not self.current_position:
            return 0.0
        
        # Basit mesafe hesaplama (Haversine formula kullanılabilir)
        lat_diff = abs(self.current_position['latitude'] - self.start_position['latitude'])
        lon_diff = abs(self.current_position['longitude'] - self.start_position['longitude'])
        
        # Yaklaşık mesafe (metre)
        distance = ((lat_diff * 111000)**2 + (lon_diff * 111000)**2)**0.5
        return distance
    
    def publish_status(self, event):
        """Sistem durumu yayınla"""
        status = {
            "state": self.competition_state,
            "task": self.current_task,
            "score": self.total_score,
            "time": time.time() - self.start_time if self.start_time else 0
        }
        
        self.mission_status_pub.publish(String(data=json.dumps(status)))
    
    def get_competition_status(self):
        """Yarışma durumu döndür"""
        return {
            "state": self.competition_state,
            "current_task": self.current_task,
            "total_score": self.total_score,
            "completed_tasks": [name for name, task in self.competition_tasks.items() if task["completed"]],
            "running_time": time.time() - self.start_time if self.start_time else 0
        }

def main():
    """Test fonksiyonu"""
    try:
        competition_manager = BARLASCompetitionManager()
        
        # Yarışmayı başlat
        if competition_manager.start_competition():
            rospy.loginfo("✅ Yarışma başlatıldı!")
            
            # ROS spin
            rospy.spin()
        else:
            rospy.logerr("❌ Yarışma başlatılamadı!")
            
    except KeyboardInterrupt:
        rospy.loginfo("⏹️ Yarışma manuel olarak durduruldu")
    except Exception as e:
        rospy.logerr(f"❌ Yarışma manager hatası: {e}")

if __name__ == "__main__":
    main()
