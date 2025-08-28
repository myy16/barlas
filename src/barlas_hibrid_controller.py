#!/usr/bin/env python3
"""
BARLAS Hibrit Mission Controller
Mission Planner + ROS + Dart Laser + Obstacle Avoidance
Teknofest 2025 İnsansız Kara Aracı Yarışması İçin Ana Kontrol Sistemi
Son Güncelleme: 27.12.2024
"""

import rospy
import time
import threading
import json
from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode

# Yarışma modüllerini import et
try:
    from competition_manager import BARLASCompetitionManager
    from traffic_sign_recognition import TrafficSignRecognizer
    from parkour_task_manager import ParkourTaskManager
    COMPETITION_MODULES_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Yarışma modülleri yüklenemedi: {e}")
    COMPETITION_MODULES_AVAILABLE = False

class BARLASHibridController:
    """
    WSL2 + Mission Planner hibrid sistem için ana controller
    Teknofest 2025 yarışması ile tam entegre
    """
    
    def __init__(self):
        rospy.init_node('barlas_hibrid_controller', anonymous=True)
        
        print("🎯 [BARLAS Hibrid] Controller başlatılıyor...")
        
        # === Sistem Durumları ===
        self.system_mode = "STANDBY"  # STANDBY, AUTONOMOUS, DART_TARGETING, MANUAL, COMPETITION
        self.mission_active = False
        self.emergency_stop = False
        self.competition_active = False
        
        # === Yarışma Sistemi Entegrasyonu ===
        if COMPETITION_MODULES_AVAILABLE:
            try:
                self.competition_manager = BARLASCompetitionManager()
                self.traffic_sign_recognizer = TrafficSignRecognizer()
                self.parkour_task_manager = ParkourTaskManager()
                self.competition_enabled = True
                print("✅ [Competition] Yarışma modülleri başarıyla yüklendi")
            except Exception as e:
                print(f"⚠️ [Competition] Yarışma modülü hatası: {e}")
                self.competition_enabled = False
        else:
            self.competition_enabled = False
            
        # === MAVROS Bağlantıları ===
        self.mavros_state = None
        self.mavros_state_sub = rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        
        # MAVROS servisler
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # === ROS Bridge Bağlantıları ===
        self.bridge_status_sub = rospy.Subscriber('/barlas/system/status', String, self.bridge_status_callback)
        
        # === Navigasyon Kontrolleri ===
        self.navigation_cmd_sub = rospy.Subscriber('/barlas/navigation/cmd_vel', Twist, self.navigation_cmd_callback)
        self.current_nav_cmd = Twist()
        
        # === Dart Targeting ===
        self.dart_targeting_pub = rospy.Publisher('/barlas/dart_laser/targeting_enable', Bool, queue_size=1)
        
        # === Yarışma Topic'leri ===
        if self.competition_enabled:
            self.competition_status_pub = rospy.Publisher('/barlas/competition/status', String, queue_size=1)
            self.competition_score_pub = rospy.Publisher('/barlas/competition/score', String, queue_size=1)
            self.traffic_sign_sub = rospy.Subscriber('/barlas/traffic_signs/detected', String, self.traffic_sign_callback)
        # === Ana Kontrol Timer ===
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.main_control_loop)  # 10Hz
        
        print("✅ [BARLAS Hibrid] Controller hazır!")
        if self.competition_enabled:
            print("🏁 [Competition] Teknofest yarışma modu etkin!")
    
    def traffic_sign_callback(self, msg):
        """Trafik işareti algılama callback"""
        if not self.competition_enabled or not self.competition_active:
            return
            
        try:
            sign_data = json.loads(msg.data)
            sign_type = sign_data.get('type', '')
            confidence = sign_data.get('confidence', 0.0)
            
            rospy.loginfo(f"🚦 [Traffic Sign] Detected: {sign_type} (confidence: {confidence:.2f})")
            
            # Yarışma yöneticisine bildir
            self.competition_manager.process_traffic_sign(sign_type, confidence)
            
        except Exception as e:
            rospy.logwarn(f"⚠️ [Traffic Sign] Processing error: {e}")
    
    def mavros_state_callback(self, msg):
        """MAVROS durumu güncelle"""
        self.mavros_state = msg
        
        if not msg.connected and self.mission_active:
            rospy.logwarn("⚠️  [MAVROS] Bağlantı kesildi - acil durdurma!")
            self.emergency_stop_procedure()
    
    def bridge_status_callback(self, msg):
        """ROS Bridge durumu"""
        # Bridge'den gelen sistem durumu
        if self.competition_enabled and self.competition_active:
            # Sistem durumunu yarışma yöneticisine ilet
            self.competition_manager.update_system_status(msg.data)
    
    def navigation_cmd_callback(self, msg):
        """Navigasyon komutu al"""
        self.current_nav_cmd = msg
    
    def main_control_loop(self, event):
        """Ana kontrol döngüsü - 10Hz"""
        if not self.mavros_state or not self.mavros_state.connected:
            return
            
        if self.emergency_stop:
            self.send_emergency_stop_command()
            return
        
        # Yarışma modu kontrolü
        if self.system_mode == "COMPETITION" and self.competition_enabled:
            self.execute_competition_mode()
        elif self.system_mode == "AUTONOMOUS":
            self.execute_autonomous_navigation()
        elif self.system_mode == "DART_TARGETING":
            self.execute_dart_targeting_mode()
        elif self.system_mode == "MANUAL":
            # Manuel kontrol Mission Planner'dan gelir
            pass
            
        # Yarışma durumu güncellemesi
        if self.competition_enabled and self.competition_active:
            self._publish_competition_status()
    
    def execute_competition_mode(self):
        """Teknofest yarışma modu - tam otomatik"""
        if not self.competition_active or not self.mission_active:
            return
            
        try:
            # Yarışma yöneticisinden komut al
            competition_command = self.competition_manager.get_current_command()
            
            if competition_command:
                cmd_type = competition_command.get('type', '')
                
                if cmd_type == 'navigation':
                    # Normal navigasyon komutu
                    nav_cmd = competition_command.get('cmd_vel', Twist())
                    self.current_nav_cmd = nav_cmd
                    self.execute_autonomous_navigation()
                    
                elif cmd_type == 'dart_targeting':
                    # Dart hedefleme modu
                    self.execute_dart_targeting_mode()
                    
                elif cmd_type == 'parkour_task':
                    # Parkur görev kontrolü
                    task_name = competition_command.get('task', '')
                    self.execute_parkour_task(task_name)
                    
                elif cmd_type == 'emergency_stop':
                    # Yarışma acil durumu
                    rospy.logwarn("🚨 [Competition] Emergency stop command received!")
                    self.emergency_stop_procedure()
                    
        except Exception as e:
            rospy.logerr(f"❌ [Competition Mode] Error: {e}")
            
    def execute_parkour_task(self, task_name):
        """Parkur görevini çalıştır"""
        if not self.competition_enabled:
            return
            
        try:
            # Parkur görev yöneticisinden komut al
            task_command = self.parkour_task_manager.get_task_command(task_name)
            
            if task_command:
                # Göreve özel navigasyon parametreleri
                max_speed = task_command.get('max_speed', 0.5)
                steering_gain = task_command.get('steering_gain', 1.0)
                
                # Navigasyon komutunu göreve uyarla
                adapted_cmd = Twist()
                adapted_cmd.linear.x = min(self.current_nav_cmd.linear.x, max_speed)
                adapted_cmd.angular.z = self.current_nav_cmd.angular.z * steering_gain
                
                self.current_nav_cmd = adapted_cmd
                self.execute_autonomous_navigation()
                
        except Exception as e:
            rospy.logerr(f"❌ [Parkour Task] Error: {e}")
    
    def _publish_competition_status(self):
        """Yarışma durumunu yayınla"""
        if not self.competition_enabled:
            return
            
        try:
            # Yarışma durumu
            status = self.competition_manager.get_status()
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.competition_status_pub.publish(status_msg)
            
            # Skor durumu
            score = self.competition_manager.get_score()
            score_msg = String()
            score_msg.data = json.dumps(score)
            self.competition_score_pub.publish(score_msg)
            
        except Exception as e:
            rospy.logdebug(f"Competition status publish error: {e}")
    
    def execute_autonomous_navigation(self):
        """Otonom navigasyon modu"""
        if not self.mission_active:
            return
            
        # Navigasyon komutlarını RC Override'a çevir
        rc_override = OverrideRCIn()
        
        # Twist komutunu RC kanallarına map et
        # Throttle (Channel 3): 1000-2000, center: 1500
        # Steering (Channel 1): 1000-2000, center: 1500
        
        throttle = int(1500 + (self.current_nav_cmd.linear.x * 400))  # -1,1 -> 1100,1900
        steering = int(1500 + (self.current_nav_cmd.angular.z * 400))  # -1,1 -> 1100,1900
        
        # Güvenlik limitleri
        throttle = max(1100, min(1900, throttle))
        steering = max(1100, min(1900, steering))
        
        rc_override.channels = [steering, 1500, throttle, 1500, 0, 0, 0, 0]
        self.rc_override_pub.publish(rc_override)
        
        rospy.logdebug(f"[Navigation] T:{throttle} S:{steering}")
    
    def execute_dart_targeting_mode(self):
        """Dart hedefleme modu - araç durur, pan-tilt aktif"""
        # Aracı durdur
        rc_override = OverrideRCIn()
        rc_override.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]  # Tüm kanallar merkez
        self.rc_override_pub.publish(rc_override)
        
        # Dart targeting sistemini aktif et
        dart_enable = Bool()
        dart_enable.data = True
        self.dart_targeting_pub.publish(dart_enable)
    
    def send_emergency_stop_command(self):
        """Acil durdurma komutu"""
        rc_override = OverrideRCIn()
        rc_override.channels = [1500, 1500, 1000, 1500, 0, 0, 0, 0]  # Throttle minimum
        self.rc_override_pub.publish(rc_override)
    
    # === Sistem Kontrol API ===
    
    def start_competition(self):
        """Teknofest yarışmasını başlat"""
        if not self.competition_enabled:
            rospy.logerr("❌ [Competition] Yarışma modülleri mevcut değil!")
            return False
            
        if not self.mavros_state or not self.mavros_state.connected:
            rospy.logerr("❌ [Competition] MAVROS bağlantısı yok!")
            return False
        
        try:
            # Yarışma yöneticisini başlat
            success = self.competition_manager.start_competition()
            if not success:
                rospy.logerr("❌ [Competition] Yarışma başlatılamadı!")
                return False
            
            # Aracı arm et
            arm_result = self.arming_client(True)
            if arm_result.success:
                self.mission_active = True
                self.competition_active = True
                self.system_mode = "COMPETITION"
                rospy.loginfo("🏁 [Competition] TEKNOFEST YARIŞMASI BAŞLATILDI!")
                return True
            else:
                rospy.logerr("❌ [Competition] Araç arm edilemedi!")
                self.competition_manager.stop_competition()
                return False
                
        except Exception as e:
            rospy.logerr(f"❌ [Competition] Başlatma hatası: {e}")
            return False
    
    def stop_competition(self):
        """Yarışmayı durdur"""
        self.mission_active = False
        self.competition_active = False
        self.system_mode = "STANDBY"
        
        if self.competition_enabled:
            self.competition_manager.stop_competition()
        
        # Disarm the vehicle
        self.arming_client(False)
        
        # Stop all movement
        rc_override = OverrideRCIn()
        rc_override.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
        self.rc_override_pub.publish(rc_override)
        
        rospy.loginfo("🏁 [Competition] YARIŞMA DURDURULDU")
    
    def start_mission(self):
        """Normal görevi başlat (yarışma dışı)"""
        if not self.mavros_state or not self.mavros_state.connected:
            rospy.logerr("❌ [Mission] MAVROS bağlantısı yok!")
            return False
        
        # Arm the vehicle
        arm_result = self.arming_client(True)
        if arm_result.success:
            self.mission_active = True
            self.system_mode = "AUTONOMOUS"
            rospy.loginfo("✅ [Mission] Görev BAŞLATILDI - Otonom mod aktif")
            return True
        else:
            rospy.logerr("❌ [Mission] Arm edilemedi!")
            return False
    
    def stop_mission(self):
        """Görevi durdur"""
        self.mission_active = False
        self.system_mode = "STANDBY"
        
        # Disarm the vehicle
        self.arming_client(False)
        
        # Stop all movement
        rc_override = OverrideRCIn()
        rc_override.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
        self.rc_override_pub.publish(rc_override)
        
        rospy.loginfo("🛑 [Mission] Görev DURDURULDU")
    
    def switch_to_dart_targeting(self):
        """Dart hedefleme moduna geç"""
        self.system_mode = "DART_TARGETING"
        rospy.loginfo("🎯 [Mode] Dart hedefleme moduna geçildi")
    
    def switch_to_autonomous(self):
        """Otonom moduna geri dön"""
        if self.competition_active:
            self.system_mode = "COMPETITION"
        else:
            self.system_mode = "AUTONOMOUS"
        
        # Dart targeting'i kapat
        dart_disable = Bool()
        dart_disable.data = False
        self.dart_targeting_pub.publish(dart_disable)
        
        mode_text = "yarışma" if self.competition_active else "otonom"
        rospy.loginfo(f"🚗 [Mode] {mode_text.capitalize()} moda geri dönüldü")
    
    def switch_to_manual(self):
        """Manuel moda geç (Mission Planner kontrolü)"""
        self.system_mode = "MANUAL"
        
        # RC Override'ı temizle
        rc_override = OverrideRCIn()
        rc_override.channels = [0, 0, 0, 0, 0, 0, 0, 0]  # Clear all overrides
        self.rc_override_pub.publish(rc_override)
        
        rospy.loginfo("🎮 [Mode] Manuel moda geçildi - Mission Planner kontrolü")
    
    def emergency_stop_procedure(self):
        """Acil durdurma prosedürü"""
        self.emergency_stop = True
        self.mission_active = False
        self.competition_active = False
        self.system_mode = "EMERGENCY"
        
        rospy.logerr("🚨 [EMERGENCY] ACİL DURDURMA AKTİF!")
        
        # Immediate stop
        self.send_emergency_stop_command()
        
        # Disarm
        self.arming_client(False)
        
        # Yarışmayı durdur
        if self.competition_enabled:
            self.competition_manager.emergency_stop()
    
    def clear_emergency_stop(self):
        """Acil durdurma temizle"""
        self.emergency_stop = False
        self.system_mode = "STANDBY"
        rospy.loginfo("✅ [Recovery] Acil durdurma temizlendi")
    
    def get_system_status(self):
        """Sistem durumu raporu"""
        status = {
            'mode': self.system_mode,
            'mission_active': self.mission_active,
            'emergency_stop': self.emergency_stop,
            'competition_active': self.competition_active,
            'competition_enabled': self.competition_enabled,
            'mavros_connected': self.mavros_state.connected if self.mavros_state else False,
            'armed': self.mavros_state.armed if self.mavros_state else False
        }
        
        # Yarışma durumu ekle
        if self.competition_enabled and self.competition_active:
            try:
                comp_status = self.competition_manager.get_status()
                comp_score = self.competition_manager.get_score()
                status['competition_status'] = comp_status
                status['competition_score'] = comp_score
            except:
                pass
                
        return status
    
    def get_competition_report(self):
        """Detaylı yarışma raporu"""
        if not self.competition_enabled:
            return {"error": "Competition not enabled"}
            
        try:
            return {
                "competition_status": self.competition_manager.get_status(),
                "score": self.competition_manager.get_score(),
                "tasks_completed": self.competition_manager.get_completed_tasks(),
                "current_task": self.competition_manager.get_current_task(),
                "time_remaining": self.competition_manager.get_time_remaining(),
                "traffic_signs_detected": self.traffic_sign_recognizer.get_detection_history()
            }
        except Exception as e:
            return {"error": f"Competition report error: {e}"}

def main():
    try:
        controller = BARLASHibridController()
        
        print("🎯 BARLAS Hibrid Mission Controller Başlatıldı!")
        print("=" * 50)
        
        if controller.competition_enabled:
            print("🏁 TEKNOFEST YARIŞMA MODU AKTİF!")
            print("Yarışma Komutları:")
            print("  Yarışma Başlat: competition")
            print("  Yarışma Durdur: stop_competition")
            print("  Yarışma Raporu: competition_report")
            print("")
            
        print("Temel Komutlar:")
        print("  Görev Başlat: start")
        print("  Görev Durdur: stop")
        print("  Dart Targeting: dart")
        print("  Otonom Mod: auto")
        print("  Manuel Mod: manual")
        print("  Acil Durdur: emergency")
        print("  Sistem Durumu: status")
        print("  Çıkış: quit")
        print("=" * 50)
        
        # Test için basit kontrol interface
        def control_interface():
            while not rospy.is_shutdown():
                try:
                    cmd = input("\n[BARLAS] Komut: ").strip().lower()
                    
                    if cmd == 'competition':
                        if controller.competition_enabled:
                            success = controller.start_competition()
                            print(f"Yarışma Başlatma: {'✅ Başarılı' if success else '❌ Başarısız'}")
                        else:
                            print("❌ Yarışma modülü mevcut değil!")
                            
                    elif cmd == 'stop_competition':
                        controller.stop_competition()
                        print("🏁 Yarışma durduruldu")
                        
                    elif cmd == 'competition_report':
                        if controller.competition_enabled:
                            report = controller.get_competition_report()
                            print(f"Yarışma Raporu: {json.dumps(report, indent=2, ensure_ascii=False)}")
                        else:
                            print("❌ Yarışma modülü mevcut değil!")
                            
                    elif cmd == 'start':
                        success = controller.start_mission()
                        print(f"Görev Başlatma: {'✅ Başarılı' if success else '❌ Başarısız'}")
                        
                    elif cmd == 'stop':
                        controller.stop_mission()
                        print("🛑 Görev durduruldu")
                        
                    elif cmd == 'dart':
                        controller.switch_to_dart_targeting()
                        print("🎯 Dart hedefleme modu aktif")
                        
                    elif cmd == 'auto':
                        controller.switch_to_autonomous()
                        print("🚗 Otonom mod aktif")
                        
                    elif cmd == 'manual':
                        controller.switch_to_manual()
                        print("🎮 Manuel mod aktif")
                        
                    elif cmd == 'emergency':
                        controller.emergency_stop_procedure()
                        print("🚨 Acil durdurma aktif")
                        
                    elif cmd == 'status':
                        status = controller.get_system_status()
                        print(f"Sistem Durumu: {json.dumps(status, indent=2, ensure_ascii=False)}")
                        
                    elif cmd == 'quit' or cmd == 'exit':
                        break
                        
                    elif cmd == 'help':
                        print("Mevcut komutlar: competition, stop_competition, competition_report, start, stop, dart, auto, manual, emergency, status, quit")
                        
                    else:
                        print("Geçersiz komut! 'help' yazın.")
                        
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"❌ Komut hatası: {e}")
        
        # Control interface'i ayrı thread'de çalıştır
        control_thread = threading.Thread(target=control_interface)
        control_thread.daemon = True
        control_thread.start()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("🛑 [BARLAS Hibrid] Sistem kapatılıyor...")
    except Exception as e:
        print(f"❌ [BARLAS Hibrid] HATA: {e}")

if __name__ == '__main__':
    main()
