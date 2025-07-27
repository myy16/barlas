#!/usr/bin/env python3
"""
BARLAS Dart Laser ROS Node
ROS ile dart tanıma ve lazer hedefleme sistemi
"""
import rospy
import cv2
import numpy as np
import threading
import time
from std_msgs.msg import String, Bool, Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge, CvBridgeError

# BARLAS dart laser sistem import
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dart_laser_targeting import DartLaserTargetingSystem, LaserPanTiltController

# Custom messages (create these in msg folder)
# from dart_laser_msgs.msg import DartDetection, LaserCommand

class DartLaserROSNode:
    """
    ROS Node for BARLAS Dart Laser System
    Dart detection, tracking ve laser targeting için ROS interface
    """
    
    def __init__(self):
        rospy.init_node('barlas_dart_laser_node', anonymous=True)
        rospy.loginfo("[DartLaserROS] BARLAS Dart Laser ROS Node başlatılıyor...")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Dart Laser System
        try:
            self.dart_laser_system = DartLaserTargetingSystem(camera_index=0)
            rospy.loginfo("[DartLaserROS] ✅ Dart Laser sistem yüklendi")
        except Exception as e:
            rospy.logerr(f"[DartLaserROS] ❌ Sistem yükleme hatası: {e}")
            return
        
        # ROS Publishers
        self.image_pub = rospy.Publisher('/barlas/dart_laser/image_result', Image, queue_size=1)
        self.compressed_pub = rospy.Publisher('/barlas/dart_laser/image_compressed', CompressedImage, queue_size=1)
        self.dart_detection_pub = rospy.Publisher('/barlas/dart_laser/dart_detected', PointStamped, queue_size=10)
        self.laser_status_pub = rospy.Publisher('/barlas/dart_laser/laser_status', Bool, queue_size=1)
        self.servo_position_pub = rospy.Publisher('/barlas/dart_laser/servo_position', Float32MultiArray, queue_size=1)
        self.system_status_pub = rospy.Publisher('/barlas/dart_laser/system_status', String, queue_size=1)
        
        # ROS Subscribers  
        self.camera_sub = rospy.Subscriber('/barlas/camera/image_raw', Image, self.camera_callback)
        self.laser_cmd_sub = rospy.Subscriber('/barlas/dart_laser/laser_command', Bool, self.laser_command_callback)
        self.servo_cmd_sub = rospy.Subscriber('/barlas/dart_laser/servo_command', Float32MultiArray, self.servo_command_callback)
        self.targeting_cmd_sub = rospy.Subscriber('/barlas/dart_laser/targeting_enable', Bool, self.targeting_enable_callback)
        
        # ROS Services (optional - for more complex commands)
        # self.calibrate_srv = rospy.Service('/barlas/dart_laser/calibrate', Empty, self.calibrate_service)
        
        # System state
        self.is_targeting_enabled = False
        self.last_dart_detection = None
        self.system_active = False
        
        # Timer for status publishing
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self.publish_status)  # 10Hz
        
        rospy.loginfo("[DartLaserROS] 🎯 ROS Node hazır - topic'ler aktif")
        
        # Ana döngüyü başlat
        self.run()
    
    def camera_callback(self, data):
        """Kamera görüntüsü callback - dart detection yapar"""
        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            if self.is_targeting_enabled:
                # Dart detection yap
                detections = self.dart_laser_system.yolo_detector.get_detections(cv_image)
                
                # En iyi dart'ı bul
                if detections:
                    best_dart = max(detections, key=lambda d: d['confidence'])
                    
                    if best_dart['confidence'] >= self.dart_laser_system.target_confidence_threshold:
                        x, y, w, h = best_dart['bbox']
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # Dart detection publish et
                        dart_point = PointStamped()
                        dart_point.header.stamp = rospy.Time.now()
                        dart_point.header.frame_id = "camera_frame"
                        dart_point.point.x = center_x
                        dart_point.point.y = center_y
                        dart_point.point.z = best_dart['confidence']  # Confidence'ı z olarak gönder
                        
                        self.dart_detection_pub.publish(dart_point)
                        self.last_dart_detection = dart_point
                        
                        # Otomatik hedefleme aktifse lazer'i yönlendir
                        if self.system_active:
                            self.dart_laser_system.laser_pantilt.aim_at_pixel(
                                center_x, center_y, cv_image.shape[1], cv_image.shape[0]
                            )
                
                # Görsel sonucu çiz
                result_image = self.dart_laser_system._draw_targeting_info(cv_image, detections)
                
                # Görüntüyü publish et
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
                    self.image_pub.publish(image_msg)
                    
                    # Compressed de gönder (bandwidth için)
                    _, compressed_data = cv2.imencode('.jpg', result_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = rospy.Time.now()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = compressed_data.tobytes()
                    self.compressed_pub.publish(compressed_msg)
                    
                except CvBridgeError as e:
                    rospy.logerr(f"[DartLaserROS] CV Bridge hatası: {e}")
        
        except Exception as e:
            rospy.logerr(f"[DartLaserROS] Kamera callback hatası: {e}")
    
    def laser_command_callback(self, data):
        """Lazer kontrol komutu callback"""
        try:
            if data.data:
                self.dart_laser_system.laser_pantilt.enable_laser()
                rospy.loginfo("[DartLaserROS] 🔴 Lazer aktif edildi")
            else:
                self.dart_laser_system.laser_pantilt.disable_laser()
                rospy.loginfo("[DartLaserROS] ⚫ Lazer kapatıldı")
            
            # Lazer durumunu publish et
            laser_status = Bool()
            laser_status.data = self.dart_laser_system.laser_pantilt.laser_active
            self.laser_status_pub.publish(laser_status)
            
        except Exception as e:
            rospy.logerr(f"[DartLaserROS] Lazer komut hatası: {e}")
    
    def servo_command_callback(self, data):
        """Servo pozisyon komutu callback"""
        try:
            if len(data.data) >= 2:
                pan_angle = data.data[0]
                tilt_angle = data.data[1]
                
                self.dart_laser_system.laser_pantilt.move_to_position(pan_angle, tilt_angle)
                rospy.loginfo(f"[DartLaserROS] Servo pozisyon: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
                
        except Exception as e:
            rospy.logerr(f"[DartLaserROS] Servo komut hatası: {e}")
    
    def targeting_enable_callback(self, data):
        """Hedefleme sistemi aktif/pasif callback"""
        self.is_targeting_enabled = data.data
        self.system_active = data.data
        
        if data.data:
            rospy.loginfo("[DartLaserROS] 🎯 Hedefleme sistemi aktif")
        else:
            rospy.loginfo("[DartLaserROS] ⏸️ Hedefleme sistemi pasif")
            # Lazer'i kapat
            self.dart_laser_system.laser_pantilt.disable_laser()
    
    def publish_status(self, event):
        """Sistem durumunu publish eder (timer callback)"""
        try:
            # Servo pozisyonu
            servo_pos = Float32MultiArray()
            servo_pos.data = [
                self.dart_laser_system.laser_pantilt.pan_position,
                self.dart_laser_system.laser_pantilt.tilt_position
            ]
            self.servo_position_pub.publish(servo_pos)
            
            # Lazer durumu
            laser_status = Bool()
            laser_status.data = self.dart_laser_system.laser_pantilt.laser_active
            self.laser_status_pub.publish(laser_status)
            
            # Sistem durumu
            status_msg = String()
            status_info = {
                "targeting_enabled": self.is_targeting_enabled,
                "system_active": self.system_active,
                "laser_active": self.dart_laser_system.laser_pantilt.laser_active,
                "pan_position": self.dart_laser_system.laser_pantilt.pan_position,
                "tilt_position": self.dart_laser_system.laser_pantilt.tilt_position,
                "last_detection": self.last_dart_detection is not None
            }
            status_msg.data = str(status_info)
            self.system_status_pub.publish(status_msg)
            
        except Exception as e:
            rospy.logerr(f"[DartLaserROS] Status publish hatası: {e}")
    
    def run(self):
        """Ana ROS döngüsü"""
        rospy.loginfo("[DartLaserROS] 🚀 ROS Node çalışıyor...")
        
        try:
            # ROS spin - callback'leri işle
            rospy.spin()
            
        except rospy.ROSInterruptException:
            rospy.loginfo("[DartLaserROS] ROS Node kapatılıyor...")
        
        finally:
            # Temizlik
            self.cleanup()
    
    def cleanup(self):
        """ROS node temizliği"""
        rospy.loginfo("[DartLaserROS] Sistem temizliği...")
        
        try:
            # Lazer'i kapat
            self.dart_laser_system.laser_pantilt.disable_laser()
            
            # Servo'yu merkeze getir
            self.dart_laser_system.laser_pantilt.center_position()
            
            # Sistem temizliği
            self.dart_laser_system.cleanup()
            
        except Exception as e:
            rospy.logerr(f"[DartLaserROS] Temizlik hatası: {e}")
        
        rospy.loginfo("[DartLaserROS] 🏁 ROS Node kapatıldı")


def main():
    """Ana ROS node fonksiyonu"""
    try:
        dart_laser_node = DartLaserROSNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node başlatılamadı")
    except Exception as e:
        rospy.logerr(f"ROS Node hatası: {e}")


if __name__ == '__main__':
    main()
