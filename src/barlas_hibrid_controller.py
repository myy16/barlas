#!/usr/bin/env python3
"""
BARLAS Hibrit Mission Controller
Mission Planner + ROS + Dart Laser + Obstacle Avoidance
Teknofest Yarışması İçin Ana Kontrol Sistemi
"""

import rospy
import time
import threading
from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode

class BARLASHibridController:
    """
    WSL2 + Mission Planner hibrid sistem için ana controller
    """
    
    def __init__(self):
        rospy.init_node('barlas_hibrid_controller', anonymous=True)
        
        print("🎯 [BARLAS Hibrid] Controller başlatılıyor...")
        
        # === Sistem Durumları ===
        self.system_mode = "STANDBY"  # STANDBY, AUTONOMOUS, DART_TARGETING, MANUAL
        self.mission_active = False
        self.emergency_stop = False
        
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
        
        # === Ana Kontrol Timer ===
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.main_control_loop)  # 10Hz
        
        print("✅ [BARLAS Hibrid] Controller hazır!")
    
    def mavros_state_callback(self, msg):
        """MAVROS durumu güncelle"""
        self.mavros_state = msg
        
        if not msg.connected and self.mission_active:
            rospy.logwarn("⚠️  [MAVROS] Bağlantı kesildi - acil durdurma!")
            self.emergency_stop_procedure()
    
    def bridge_status_callback(self, msg):
        """ROS Bridge durumu"""
        # Bridge'den gelen sistem durumu
        pass
    
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
        
        if self.system_mode == "AUTONOMOUS":
            self.execute_autonomous_navigation()
        elif self.system_mode == "DART_TARGETING":
            self.execute_dart_targeting_mode()
        elif self.system_mode == "MANUAL":
            # Manuel kontrol Mission Planner'dan gelir
            pass
    
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
    
    def start_mission(self):
        """Görevi başlat"""
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
        self.system_mode = "AUTONOMOUS"
        
        # Dart targeting'i kapat
        dart_disable = Bool()
        dart_disable.data = False
        self.dart_targeting_pub.publish(dart_disable)
        
        rospy.loginfo("🚗 [Mode] Otonom moda geri dönüldü")
    
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
        self.system_mode = "EMERGENCY"
        
        rospy.logerr("🚨 [EMERGENCY] ACİL DURDURMA AKTİF!")
        
        # Immediate stop
        self.send_emergency_stop_command()
        
        # Disarm
        self.arming_client(False)
    
    def clear_emergency_stop(self):
        """Acil durdurma temizle"""
        self.emergency_stop = False
        self.system_mode = "STANDBY"
        rospy.loginfo("✅ [Recovery] Acil durdurma temizlendi")
    
    def get_system_status(self):
        """Sistem durumu raporu"""
        return {
            'mode': self.system_mode,
            'mission_active': self.mission_active,
            'emergency_stop': self.emergency_stop,
            'mavros_connected': self.mavros_state.connected if self.mavros_state else False,
            'armed': self.mavros_state.armed if self.mavros_state else False
        }

def main():
    try:
        controller = BARLASHibridController()
        
        print("🎯 BARLAS Hibrid Mission Controller Başlatıldı!")
        print("Komutlar:")
        print("  Görev Başlat: controller.start_mission()")
        print("  Görev Durdur: controller.stop_mission()")
        print("  Dart Targeting: controller.switch_to_dart_targeting()")
        print("  Otonom Mod: controller.switch_to_autonomous()")
        print("  Manuel Mod: controller.switch_to_manual()")
        print("  Acil Durdur: controller.emergency_stop_procedure()")
        
        # Test için basit kontrol interface
        def control_interface():
            while not rospy.is_shutdown():
                try:
                    cmd = input("\n[BARLAS] Komut (start/stop/dart/auto/manual/emergency/status/quit): ").strip().lower()
                    
                    if cmd == 'start':
                        controller.start_mission()
                    elif cmd == 'stop':
                        controller.stop_mission()
                    elif cmd == 'dart':
                        controller.switch_to_dart_targeting()
                    elif cmd == 'auto':
                        controller.switch_to_autonomous()
                    elif cmd == 'manual':
                        controller.switch_to_manual()
                    elif cmd == 'emergency':
                        controller.emergency_stop_procedure()
                    elif cmd == 'status':
                        status = controller.get_system_status()
                        print(f"Sistem Durumu: {status}")
                    elif cmd == 'quit':
                        break
                    else:
                        print("Geçersiz komut!")
                        
                except KeyboardInterrupt:
                    break
        
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
