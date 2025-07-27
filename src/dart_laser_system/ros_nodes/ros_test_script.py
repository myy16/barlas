#!/usr/bin/env python3
"""
BARLAS Dart Laser ROS Test Script
ROS node'ları test etmek için
"""
import rospy
import time
import sys
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image

class DartLaserROSTest:
    """
    ROS node test sınıfı
    """
    
    def __init__(self):
        rospy.init_node('barlas_dart_laser_test', anonymous=True)
        rospy.loginfo("[ROSTest] BARLAS Dart Laser ROS Test başlatılıyor...")
        
        # Publishers
        self.servo_position_pub = rospy.Publisher('/barlas/servo/position_command', Float32MultiArray, queue_size=1)
        self.laser_cmd_pub = rospy.Publisher('/barlas/servo/laser_command', Bool, queue_size=1)
        self.center_cmd_pub = rospy.Publisher('/barlas/servo/center_command', Bool, queue_size=1)
        self.pixel_target_pub = rospy.Publisher('/barlas/servo/pixel_target', Point, queue_size=1)
        self.targeting_enable_pub = rospy.Publisher('/barlas/dart_laser/targeting_enable', Bool, queue_size=1)
        
        # Subscribers
        self.servo_status_sub = rospy.Subscriber('/barlas/servo/current_position', Float32MultiArray, self.servo_status_callback)
        self.laser_status_sub = rospy.Subscriber('/barlas/servo/laser_status', Bool, self.laser_status_callback)
        self.dart_detection_sub = rospy.Subscriber('/barlas/dart_laser/dart_detected', PointStamped, self.dart_detection_callback)
        
        # Status variables
        self.current_servo_position = [90, 90]
        self.laser_active = False
        self.last_dart_detection = None
        
        rospy.loginfo("[ROSTest] ✅ Test node hazır")
    
    def servo_status_callback(self, data):
        """Servo durumu callback"""
        if len(data.data) >= 2:
            self.current_servo_position = [data.data[0], data.data[1]]
            rospy.loginfo(f"[ROSTest] Servo: Pan={data.data[0]:.1f}°, Tilt={data.data[1]:.1f}°")
    
    def laser_status_callback(self, data):
        """Lazer durumu callback"""
        self.laser_active = data.data
        status = "AKTİF" if data.data else "KAPALI"
        rospy.loginfo(f"[ROSTest] Lazer: {status}")
    
    def dart_detection_callback(self, data):
        """Dart tespit callback"""
        self.last_dart_detection = data
        rospy.loginfo(f"[ROSTest] 🎯 Dart tespit: ({data.point.x:.0f}, {data.point.y:.0f}), Güven: {data.point.z:.2f}")
    
    def test_servo_movement(self):
        """Servo hareket testi"""
        rospy.loginfo("\n=== SERVO HAREKET TESTİ ===")
        
        # Test pozisyonları
        test_positions = [
            [90, 90],   # Merkez
            [60, 60],   # Sol-aşağı
            [120, 60],  # Sağ-aşağı
            [120, 120], # Sağ-yukarı
            [60, 120],  # Sol-yukarı
            [90, 90]    # Merkez'e dön
        ]
        
        for i, (pan, tilt) in enumerate(test_positions):
            rospy.loginfo(f"Test {i+1}: Pan={pan}°, Tilt={tilt}°")
            
            # Servo pozisyon komutu gönder
            position_msg = Float32MultiArray()
            position_msg.data = [pan, tilt]
            self.servo_position_pub.publish(position_msg)
            
            time.sleep(2)  # Hareket tamamlanmasını bekle
        
        rospy.loginfo("✅ Servo hareket testi tamamlandı")
    
    def test_laser_control(self):
        """Lazer kontrol testi"""
        rospy.loginfo("\n=== LAZER KONTROL TESTİ ===")
        
        # Lazer açma
        rospy.loginfo("Lazer açılıyor...")
        laser_cmd = Bool()
        laser_cmd.data = True
        self.laser_cmd_pub.publish(laser_cmd)
        time.sleep(2)
        
        # Lazer kapatma
        rospy.loginfo("Lazer kapatılıyor...")
        laser_cmd.data = False
        self.laser_cmd_pub.publish(laser_cmd)
        time.sleep(1)
        
        rospy.loginfo("✅ Lazer kontrol testi tamamlandı")
    
    def test_pixel_targeting(self):
        """Piksel hedefleme testi"""
        rospy.loginfo("\n=== PİKSEL HEDEFLEME TESTİ ===")
        
        # Test piksel koordinatları (640x480 kamera için)
        test_pixels = [
            (320, 240),  # Merkez
            (160, 120),  # Sol-üst
            (480, 120),  # Sağ-üst
            (480, 360),  # Sağ-alt
            (160, 360),  # Sol-alt
            (320, 240)   # Merkez'e dön
        ]
        
        for i, (x, y) in enumerate(test_pixels):
            rospy.loginfo(f"Piksel hedef {i+1}: ({x}, {y})")
            
            # Piksel hedef komutu gönder
            pixel_target = Point()
            pixel_target.x = x
            pixel_target.y = y
            pixel_target.z = 0  # Frame boyutu bilgisi (opsiyonel)
            self.pixel_target_pub.publish(pixel_target)
            
            time.sleep(2)
        
        rospy.loginfo("✅ Piksel hedefleme testi tamamlandı")
    
    def test_center_command(self):
        """Merkez komutu testi"""
        rospy.loginfo("\n=== MERKEZ KOMUTU TESTİ ===")
        
        center_cmd = Bool()
        center_cmd.data = True
        self.center_cmd_pub.publish(center_cmd)
        
        time.sleep(2)
        rospy.loginfo("✅ Merkez komutu testi tamamlandı")
    
    def test_dart_targeting_system(self):
        """Dart hedefleme sistemi testi"""
        rospy.loginfo("\n=== DART HEDEFLEME SİSTEM TESTİ ===")
        
        # Hedefleme sistemini aktif et
        rospy.loginfo("Dart hedefleme sistemi aktif ediliyor...")
        targeting_enable = Bool()
        targeting_enable.data = True
        self.targeting_enable_pub.publish(targeting_enable)
        
        # 10 saniye bekle ve dart tespitlerini izle
        rospy.loginfo("10 saniye dart tespit bekleniyor...")
        start_time = time.time()
        detections_count = 0
        
        while (time.time() - start_time) < 10:
            if self.last_dart_detection:
                detections_count += 1
                self.last_dart_detection = None
            time.sleep(0.1)
        
        # Hedefleme sistemini kapat
        targeting_enable.data = False
        self.targeting_enable_pub.publish(targeting_enable)
        
        rospy.loginfo(f"✅ Dart hedefleme testi tamamlandı - {detections_count} tespit")
    
    def interactive_test(self):
        """İnteraktif test modu"""
        rospy.loginfo("\n🎯 BARLAS ROS İnteraktif Test Modu")
        
        while not rospy.is_shutdown():
            print("\n" + "="*50)
            print("BARLAS Dart Laser ROS Test Menüsü:")
            print("1. Servo Hareket Testi")
            print("2. Lazer Kontrol Testi")
            print("3. Piksel Hedefleme Testi")
            print("4. Merkez Komutu Testi")
            print("5. Dart Hedefleme Sistem Testi")
            print("6. Manuel Servo Kontrolü")
            print("7. Sistem Durumu")
            print("0. Çıkış")
            
            try:
                choice = input("\nSeçiminiz (0-7): ").strip()
                
                if choice == '1':
                    self.test_servo_movement()
                elif choice == '2':
                    self.test_laser_control()
                elif choice == '3':
                    self.test_pixel_targeting()
                elif choice == '4':
                    self.test_center_command()
                elif choice == '5':
                    self.test_dart_targeting_system()
                elif choice == '6':
                    self.manual_servo_control()
                elif choice == '7':
                    self.show_system_status()
                elif choice == '0':
                    rospy.loginfo("Test sonlandırılıyor...")
                    break
                else:
                    print("❌ Geçersiz seçim!")
                    
            except KeyboardInterrupt:
                rospy.loginfo("\nTest iptal edildi")
                break
            except Exception as e:
                rospy.logerr(f"Test hatası: {e}")
    
    def manual_servo_control(self):
        """Manuel servo kontrolü"""
        rospy.loginfo("\n=== MANUEL SERVO KONTROLÜ ===")
        print("Komutlar: w/s (tilt), a/d (pan), l (laser), c (center), q (çıkış)")
        
        while True:
            try:
                command = input("Komut: ").lower().strip()
                
                if command == 'q':
                    break
                elif command == 'w':
                    new_tilt = min(150, self.current_servo_position[1] + 5)
                    self.send_servo_position(self.current_servo_position[0], new_tilt)
                elif command == 's':
                    new_tilt = max(30, self.current_servo_position[1] - 5)
                    self.send_servo_position(self.current_servo_position[0], new_tilt)
                elif command == 'a':
                    new_pan = max(10, self.current_servo_position[0] - 5)
                    self.send_servo_position(new_pan, self.current_servo_position[1])
                elif command == 'd':
                    new_pan = min(170, self.current_servo_position[0] + 5)
                    self.send_servo_position(new_pan, self.current_servo_position[1])
                elif command == 'l':
                    laser_cmd = Bool()
                    laser_cmd.data = not self.laser_active
                    self.laser_cmd_pub.publish(laser_cmd)
                elif command == 'c':
                    self.test_center_command()
                else:
                    print("❌ Bilinmeyen komut!")
                    
            except KeyboardInterrupt:
                break
    
    def send_servo_position(self, pan, tilt):
        """Servo pozisyon gönder"""
        position_msg = Float32MultiArray()
        position_msg.data = [pan, tilt]
        self.servo_position_pub.publish(position_msg)
    
    def show_system_status(self):
        """Sistem durumunu göster"""
        rospy.loginfo("\n=== SİSTEM DURUMU ===")
        print(f"Servo Pozisyon: Pan={self.current_servo_position[0]:.1f}°, Tilt={self.current_servo_position[1]:.1f}°")
        print(f"Lazer Durumu: {'AKTİF' if self.laser_active else 'KAPALI'}")
        print(f"Son Dart Tespit: {'VAR' if self.last_dart_detection else 'YOK'}")
        print(f"ROS Bağlantı: {'AKTİF' if not rospy.is_shutdown() else 'KAPALI'}")


def main():
    """Ana test fonksiyonu"""
    try:
        rospy.loginfo("🎯 BARLAS Dart Laser ROS Test başlatılıyor...")
        
        # Test node'u oluştur
        test_node = DartLaserROSTest()
        
        # Kısa bekleme (node'ların bağlanması için)
        time.sleep(2)
        
        if len(sys.argv) > 1:
            # Komut satırı argümanı varsa o testi çalıştır
            test_type = sys.argv[1]
            
            if test_type == "servo":
                test_node.test_servo_movement()
            elif test_type == "laser":
                test_node.test_laser_control()
            elif test_type == "pixel":
                test_node.test_pixel_targeting()
            elif test_type == "center":
                test_node.test_center_command()
            elif test_type == "dart":
                test_node.test_dart_targeting_system()
            else:
                rospy.logwarn(f"Bilinmeyen test türü: {test_type}")
        else:
            # İnteraktif mod
            test_node.interactive_test()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Test iptal edildi")
    except Exception as e:
        rospy.logerr(f"Test hatası: {e}")
    finally:
        rospy.loginfo("🏁 ROS Test tamamlandı")


if __name__ == '__main__':
    main()
