#!/usr/bin/env python3
"""
BARLAS Servo Controller ROS Node
Servo motorları ROS ile kontrol etmek için özel node
"""
import rospy
import time
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Point

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    rospy.logwarn("RPi.GPIO bulunamadı - simülasyon modu")

class ServoControllerROSNode:
    """
    ROS Node for servo motor control
    Pan-Tilt servo sistemi için özel ROS interface
    """
    
    def __init__(self):
        rospy.init_node('barlas_servo_controller', anonymous=True)
        rospy.loginfo("[ServoROS] BARLAS Servo Controller ROS Node başlatılıyor...")
        
        # GPIO pin parametreleri (ROS param'dan al)
        self.pan_pin = rospy.get_param('~pan_pin', 18)
        self.tilt_pin = rospy.get_param('~tilt_pin', 19)
        self.laser_pin = rospy.get_param('~laser_pin', 20)
        
        # Servo pozisyonları
        self.current_pan = 90.0
        self.current_tilt = 90.0
        self.laser_active = False
        
        # Servo limitleri
        self.pan_min = rospy.get_param('~pan_min', 10)
        self.pan_max = rospy.get_param('~pan_max', 170)
        self.tilt_min = rospy.get_param('~tilt_min', 30)
        self.tilt_max = rospy.get_param('~tilt_max', 150)
        
        # PWM nesneleri
        self.pan_pwm = None
        self.tilt_pwm = None
        
        # GPIO kurulumu
        if RPI_AVAILABLE:
            self.setup_gpio()
        else:
            rospy.logwarn("[ServoROS] Simülasyon modu - gerçek servo yok")
        
        # ROS Publishers
        self.position_pub = rospy.Publisher('/barlas/servo/current_position', Float32MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher('/barlas/servo/status', String, queue_size=1)
        self.laser_status_pub = rospy.Publisher('/barlas/servo/laser_status', Bool, queue_size=1)
        
        # ROS Subscribers
        self.position_cmd_sub = rospy.Subscriber('/barlas/servo/position_command', Float32MultiArray, self.position_command_callback)
        self.laser_cmd_sub = rospy.Subscriber('/barlas/servo/laser_command', Bool, self.laser_command_callback)
        self.center_cmd_sub = rospy.Subscriber('/barlas/servo/center_command', Bool, self.center_command_callback)
        self.pixel_target_sub = rospy.Subscriber('/barlas/servo/pixel_target', Point, self.pixel_target_callback)
        
        # Timer for status publishing
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self.publish_status)  # 10Hz
        
        # Başlangıç pozisyonu
        self.move_to_position(self.current_pan, self.current_tilt)
        
        rospy.loginfo(f"[ServoROS] ✅ Servo Controller hazır - Pin: Pan={self.pan_pin}, Tilt={self.tilt_pin}, Laser={self.laser_pin}")
    
    def setup_gpio(self):
        """GPIO pinlerini ve PWM'i ayarlar"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pan_pin, GPIO.OUT)
            GPIO.setup(self.tilt_pin, GPIO.OUT)
            GPIO.setup(self.laser_pin, GPIO.OUT)
            
            # PWM ayarları (50Hz servo için)
            self.pan_pwm = GPIO.PWM(self.pan_pin, 50)
            self.tilt_pwm = GPIO.PWM(self.tilt_pin, 50)
            
            self.pan_pwm.start(0)
            self.tilt_pwm.start(0)
            
            # Lazer başlangıçta kapalı
            GPIO.output(self.laser_pin, GPIO.LOW)
            
            rospy.loginfo("[ServoROS] ✅ GPIO kurulumu tamamlandı")
            
        except Exception as e:
            rospy.logerr(f"[ServoROS] GPIO kurulum hatası: {e}")
    
    def angle_to_duty_cycle(self, angle):
        """Servo açısını PWM duty cycle'a çevirir"""
        return 2.5 + (angle / 180.0) * 10.0
    
    def move_to_position(self, pan_angle, tilt_angle):
        """Belirtilen açılara hareket eder"""
        
        # Açı sınırlarını kontrol et
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        self.current_pan = pan_angle
        self.current_tilt = tilt_angle
        
        if RPI_AVAILABLE and self.pan_pwm and self.tilt_pwm:
            try:
                pan_duty = self.angle_to_duty_cycle(pan_angle)
                tilt_duty = self.angle_to_duty_cycle(tilt_angle)
                
                self.pan_pwm.ChangeDutyCycle(pan_duty)
                self.tilt_pwm.ChangeDutyCycle(tilt_duty)
                
                time.sleep(0.2)  # Servo hareket süresi
                
                # PWM'i durdur (servo titremesini önler)
                self.pan_pwm.ChangeDutyCycle(0)
                self.tilt_pwm.ChangeDutyCycle(0)
                
            except Exception as e:
                rospy.logerr(f"[ServoROS] Servo hareket hatası: {e}")
        
        rospy.loginfo(f"[ServoROS] Pozisyon: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
    
    def enable_laser(self):
        """Lazer pointer'ı açar"""
        if RPI_AVAILABLE:
            try:
                GPIO.output(self.laser_pin, GPIO.HIGH)
                self.laser_active = True
                rospy.loginfo("[ServoROS] 🔴 LAZER AKTİF")
            except Exception as e:
                rospy.logerr(f"[ServoROS] Lazer açma hatası: {e}")
        else:
            self.laser_active = True
            rospy.loginfo("[ServoROS] 🔴 LAZER AKTİF (Simülasyon)")
    
    def disable_laser(self):
        """Lazer pointer'ı kapatır"""
        if RPI_AVAILABLE:
            try:
                GPIO.output(self.laser_pin, GPIO.LOW)
                self.laser_active = False
                rospy.loginfo("[ServoROS] ⚫ Lazer kapatıldı")
            except Exception as e:
                rospy.logerr(f"[ServoROS] Lazer kapatma hatası: {e}")
        else:
            self.laser_active = False
            rospy.loginfo("[ServoROS] ⚫ Lazer kapatıldı (Simülasyon)")
    
    def pixel_to_angle(self, pixel_x, pixel_y, frame_width=640, frame_height=480):
        """Piksel koordinatlarını servo açılarına çevirir"""
        
        # Merkez noktasından fark
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        offset_x = pixel_x - center_x
        offset_y = pixel_y - center_y
        
        # Piksel farkını açıya çevir (kamera FOV'a göre)
        horizontal_fov = 60
        vertical_fov = 45
        
        pan_adjustment = (offset_x / center_x) * (horizontal_fov / 2)
        tilt_adjustment = -(offset_y / center_y) * (vertical_fov / 2)  # Y ekseni ters
        
        # Mevcut pozisyona ekle
        target_pan = self.current_pan + pan_adjustment
        target_tilt = self.current_tilt + tilt_adjustment
        
        return target_pan, target_tilt
    
    def position_command_callback(self, data):
        """Servo pozisyon komutu callback"""
        try:
            if len(data.data) >= 2:
                pan_angle = data.data[0]
                tilt_angle = data.data[1]
                self.move_to_position(pan_angle, tilt_angle)
                
        except Exception as e:
            rospy.logerr(f"[ServoROS] Pozisyon komut hatası: {e}")
    
    def laser_command_callback(self, data):
        """Lazer kontrol komutu callback"""
        try:
            if data.data:
                self.enable_laser()
            else:
                self.disable_laser()
                
        except Exception as e:
            rospy.logerr(f"[ServoROS] Lazer komut hatası: {e}")
    
    def center_command_callback(self, data):
        """Merkez pozisyon komutu callback"""
        if data.data:
            self.move_to_position(90, 90)
            self.disable_laser()
            rospy.loginfo("[ServoROS] 🎯 Merkez pozisyona getirildi")
    
    def pixel_target_callback(self, data):
        """Piksel hedef callback - otomatik servo yönlendirme"""
        try:
            pixel_x = data.x
            pixel_y = data.y
            frame_width = 640  # data.z olarak gelebilir
            frame_height = 480
            
            target_pan, target_tilt = self.pixel_to_angle(pixel_x, pixel_y, frame_width, frame_height)
            self.move_to_position(target_pan, target_tilt)
            
            rospy.loginfo(f"[ServoROS] Piksel hedef: ({pixel_x}, {pixel_y}) -> Pan={target_pan:.1f}°, Tilt={target_tilt:.1f}°")
            
        except Exception as e:
            rospy.logerr(f"[ServoROS] Piksel hedef hatası: {e}")
    
    def publish_status(self, event):
        """Servo durumunu publish eder"""
        try:
            # Mevcut pozisyon
            position_msg = Float32MultiArray()
            position_msg.data = [self.current_pan, self.current_tilt]
            self.position_pub.publish(position_msg)
            
            # Lazer durumu
            laser_status = Bool()
            laser_status.data = self.laser_active
            self.laser_status_pub.publish(laser_status)
            
            # Genel durum
            status_msg = String()
            status_info = {
                "pan_position": self.current_pan,
                "tilt_position": self.current_tilt,
                "laser_active": self.laser_active,
                "gpio_available": RPI_AVAILABLE,
                "timestamp": rospy.Time.now().to_sec()
            }
            status_msg.data = str(status_info)
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            rospy.logerr(f"[ServoROS] Status publish hatası: {e}")
    
    def cleanup(self):
        """GPIO temizleme"""
        rospy.loginfo("[ServoROS] Servo sistem temizliği...")
        
        # Lazer'i kapat
        self.disable_laser()
        
        # Servo'yu merkeze getir
        self.move_to_position(90, 90)
        
        if RPI_AVAILABLE and self.pan_pwm and self.tilt_pwm:
            try:
                self.pan_pwm.stop()
                self.tilt_pwm.stop()
                GPIO.cleanup()
                rospy.loginfo("[ServoROS] ✅ GPIO temizlendi")
            except Exception as e:
                rospy.logerr(f"[ServoROS] Temizleme hatası: {e}")


def main():
    """Ana ROS servo node fonksiyonu"""
    try:
        servo_node = ServoControllerROSNode()
        
        # ROS spin
        rospy.loginfo("[ServoROS] 🚀 Servo Controller çalışıyor...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("[ServoROS] ROS Node kapatılıyor...")
    
    except Exception as e:
        rospy.logerr(f"[ServoROS] Servo node hatası: {e}")
    
    finally:
        if 'servo_node' in locals():
            servo_node.cleanup()
        rospy.loginfo("[ServoROS] 🏁 Servo Controller kapatıldı")


if __name__ == '__main__':
    main()
