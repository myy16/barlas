#!/usr/bin/env python3
"""
BARLAS Robot - PX4 Kontrol Node'u
Simülasyon ve gerçek robot için PX4 kontrolü
"""
import rospy
import mavros
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from math import pi

class PX4Controller:
    def __init__(self):
        rospy.init_node('px4_control_node')
        
        # Parametreler
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.max_turn_rate = rospy.get_param('~max_turn_rate', 1.0)
        
        # MAVROS durum değişkenleri
        self.current_state = State()
        self.local_position = PoseStamped()
        
        # ROS Publishers/Subscribers
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_callback)
        
        # MAVROS Servisleri
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        
        rospy.loginfo("✅ PX4 kontrol node'u başlatıldı")
    
    def state_callback(self, msg):
        """MAVROS durum callback fonksiyonu"""
        self.current_state = msg
    
    def position_callback(self, msg):
        """Konum callback fonksiyonu"""
        self.local_position = msg
    
    def arm(self):
        """Aracı arm et"""
        if not self.current_state.armed:
            for _ in range(10):
                if self.arming_client(True).success:
                    rospy.loginfo("✅ Araç arm edildi")
                    return True
                rospy.sleep(1.0)
        return False
    
    def set_mode(self, mode):
        """Uçuş modunu değiştir"""
        if self.current_state.mode != mode:
            for _ in range(10):
                if self.set_mode_client(custom_mode=mode).mode_sent:
                    rospy.loginfo(f"✅ Mod değiştirildi: {mode}")
                    return True
                rospy.sleep(1.0)
        return False
    
    def takeoff(self, height=2.0):
        """Kalkış yap"""
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = height
        
        # Kalkış öncesi hazırlık
        for _ in range(100):
            if not rospy.is_shutdown():
                self.local_pos_pub.publish(pose)
                rospy.sleep(0.01)
        
        # OFFBOARD moda geç ve arm et
        if self.set_mode("OFFBOARD") and self.arm():
            rospy.loginfo(f"🛫 Kalkış başlatıldı - Hedef yükseklik: {height}m")
            return True
        return False
    
    def land(self):
        """İniş yap"""
        if self.set_mode("AUTO.LAND"):
            rospy.loginfo("🛬 İniş başlatıldı")
            return True
        return False
    
    def set_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Hız komutları gönder"""
        cmd = TwistStamped()
        cmd.twist.linear.x = max(min(vx, self.max_velocity), -self.max_velocity)
        cmd.twist.linear.y = max(min(vy, self.max_velocity), -self.max_velocity)
        cmd.twist.linear.z = max(min(vz, self.max_velocity), -self.max_velocity)
        cmd.twist.angular.z = max(min(yaw_rate, self.max_turn_rate), -self.max_turn_rate)
        
        self.cmd_vel_pub.publish(cmd)
    
    def goto_position(self, x, y, z, yaw=0.0):
        """Belirli bir konuma git"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Yaw açısını quaternion'a çevir
        from math import sin, cos
        yaw = yaw * pi / 180.0  # Dereceyi radyana çevir
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw/2.0)
        pose.pose.orientation.w = cos(yaw/2.0)
        
        self.local_pos_pub.publish(pose)

def test_flight():
    """Test uçuşu gerçekleştir"""
    controller = PX4Controller()
    rate = rospy.Rate(20)
    
    # Kalkış
    if not controller.takeoff(2.0):
        rospy.logerr("❌ Kalkış başarısız")
        return
    
    rospy.sleep(5)  # Kalkış için bekle
    
    # Kare şeklinde uç
    waypoints = [
        (2, 0, 2),    # İleri
        (2, 2, 2),    # Sağa
        (0, 2, 2),    # Geri
        (0, 0, 2)     # Sola
    ]
    
    for x, y, z in waypoints:
        controller.goto_position(x, y, z)
        rospy.sleep(10)  # Her nokta için bekle
    
    # İniş
    controller.land()
    rospy.spin()

if __name__ == '__main__':
    try:
        test_flight()
    except rospy.ROSInterruptException:
        pass
