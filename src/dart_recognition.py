"""
BARLAS Dart Recognition System
YOLO tabanlı dart tanıma ve takip sistemi - Pan-Tilt kamera entegrasyonu
"""
import cv2
import numpy as np
import time
import threading
from typing import Dict, List, Tuple, Optional
import os
import sys

# BARLAS modüllerini import et
from dart_recognize.yolo_predictions import YOLOPredictions
from pan_tilt_controller import PanTiltController
from motor_driver import MotorDriver

class DartRecognition:
    def __init__(self, camera_index=0, use_pan_tilt=True):
        """
        BARLAS Dart Recognition System
        
        Args:
            camera_index: Kamera indeksi (varsayılan: 0)
            use_pan_tilt: Pan-Tilt servo kullanımı (varsayılan: True)
        """
        
        # YOLO dart detector
        self.yolo_detector = YOLOPredictions()
        
        # Kamera kurulumu
        self.camera_index = camera_index
        self.cap = None
        self.is_camera_active = False
        
        # Pan-Tilt kontrol
        self.use_pan_tilt = use_pan_tilt
        self.pan_tilt = None
        if use_pan_tilt:
            try:
                self.pan_tilt = PanTiltController()
                print("Pan-Tilt controller başlatıldı")
            except Exception as e:
                print(f"Pan-Tilt controller hatası: {e}")
                self.use_pan_tilt = False
        
        # Araç motoru (dart'a yaklaşmak için)
        self.motor_driver = None
        try:
            self.motor_driver = MotorDriver()
            print("Motor driver başlatıldı")
        except Exception as e:
            print(f"Motor driver hatası: {e}")
        
        # Dart takip değişkenleri
        self.target_dart = None
        self.target_center = None
        self.tracking_active = False
        
        # Kamera frame merkezi
        self.frame_center_x = 320  # Varsayılan
        self.frame_center_y = 240  # Varsayılan
        
        # PID değerleri (pan-tilt için)
        self.pan_kp = 0.1
        self.tilt_kp = 0.1
        
        # Threading için
        self.recognition_thread = None
        self.is_running = False
        
        print("BARLAS Dart Recognition System başlatıldı")
    
    def initialize_camera(self):
        """Kamerayı başlatır"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"Kamera {self.camera_index} açılamadı!")
                return False
            
            # Kamera ayarları
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Frame merkezi güncelle
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.frame_center_x = width // 2
            self.frame_center_y = height // 2
            
            self.is_camera_active = True
            print(f"Kamera başlatıldı: {width}x{height}")
            return True
            
        except Exception as e:
            print(f"Kamera başlatma hatası: {e}")
            return False
    
    def start_recognition(self):
        """Dart tanıma sistemini başlatır"""
        if not self.initialize_camera():
            return False
        
        self.is_running = True
        self.recognition_thread = threading.Thread(target=self._recognition_loop, daemon=True)
        self.recognition_thread.start()
        
        print("Dart tanıma sistemi başlatıldı")
        return True
    
    def stop_recognition(self):
        """Dart tanıma sistemini durdurur"""
        self.is_running = False
        self.tracking_active = False
        
        if self.recognition_thread:
            self.recognition_thread.join(timeout=2.0)
        
        if self.cap:
            self.cap.release()
        
        cv2.destroyAllWindows()
        print("Dart tanıma sistemi durduruldu")
    
    def _recognition_loop(self):
        """Ana tanıma döngüsü (thread'de çalışır)"""
        
        while self.is_running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            try:
                # Dart tespiti yap
                detections = self.yolo_detector.get_detections(frame)
                
                if detections:
                    # En büyük dart'ı hedef al
                    largest_dart = max(detections, key=lambda d: d['bbox'][2] * d['bbox'][3])
                    
                    # Hedef merkezi hesapla
                    x, y, w, h = largest_dart['bbox']
                    target_x = x + w // 2
                    target_y = y + h // 2
                    
                    self.target_dart = largest_dart
                    self.target_center = (target_x, target_y)
                    
                    # Pan-Tilt kontrolü
                    if self.use_pan_tilt and self.pan_tilt:
                        self._update_pan_tilt(target_x, target_y)
                    
                    # Görsel çıktı
                    frame = self._draw_target_info(frame, largest_dart)
                
                else:
                    self.target_dart = None
                    self.target_center = None
                
                # Görüntüyü göster
                cv2.imshow('BARLAS Dart Recognition', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.is_running = False
                    break
                    
            except Exception as e:
                print(f"Tanıma döngüsü hatası: {e}")
                time.sleep(0.1)
            
            time.sleep(0.03)  # ~30 FPS
    
    def _update_pan_tilt(self, target_x, target_y):
        """Pan-Tilt servo pozisyonunu günceller"""
        
        # Merkez farkını hesapla
        error_x = target_x - self.frame_center_x
        error_y = target_y - self.frame_center_y
        
        # Minimum hareket eşiği
        if abs(error_x) < 20 and abs(error_y) < 20:
            return
        
        try:
            # Pan ayarlaması (yatay)
            pan_adjustment = -error_x * self.pan_kp  # Negatif çünkü servo ters çalışır
            self.pan_tilt.adjust_pan(pan_adjustment)
            
            # Tilt ayarlaması (dikey)
            tilt_adjustment = error_y * self.tilt_kp
            self.pan_tilt.adjust_tilt(tilt_adjustment)
            
        except Exception as e:
            print(f"Pan-Tilt güncelleme hatası: {e}")
    
    def _draw_target_info(self, frame, dart_detection):
        """Hedef bilgilerini frame üzerine çizer"""
        
        x, y, w, h = dart_detection['bbox']
        confidence = dart_detection['confidence']
        
        # Bounding box
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Merkez noktası
        center_x = x + w // 2
        center_y = y + h // 2
        cv2.circle(frame, (center_x, center_y), 8, (0, 255, 0), -1)
        
        # Frame merkezi
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), 5, (255, 0, 0), -1)
        
        # Çizgiler (merkez - hedef)
        cv2.line(frame, (self.frame_center_x, self.frame_center_y), 
                (center_x, center_y), (255, 255, 0), 2)
        
        # Bilgi metni
        info_text = f"Dart: {confidence:.2f}"
        cv2.putText(frame, info_text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Uzaklık bilgisi
        distance = np.sqrt((center_x - self.frame_center_x)**2 + (center_y - self.frame_center_y)**2)
        distance_text = f"Distance: {distance:.1f}px"
        cv2.putText(frame, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame
    
    def get_target_position(self):
        """Hedef dart pozisyonunu döndürür"""
        return self.target_center
    
    def get_target_detection(self):
        """Hedef dart detection bilgilerini döndürür"""
        return self.target_dart
    
    def is_target_acquired(self):
        """Hedef kilitlenmiş mi kontrol eder"""
        return self.target_dart is not None
    
    def move_towards_target(self, speed=50):
        """Hedefe doğru araçla hareket eder"""
        if not self.motor_driver or not self.target_center:
            return False
        
        center_x, center_y = self.target_center
        error_x = center_x - self.frame_center_x
        
        try:
            if abs(error_x) > 50:  # Hedefe henüz merkeze gelmemiş
                if error_x > 0:  # Hedef sağda
                    self.motor_driver.turn_right(speed)
                else:  # Hedef solda
                    self.motor_driver.turn_left(speed)
            else:
                # Düz ilerle
                self.motor_driver.move_forward(speed)
            
            return True
            
        except Exception as e:
            print(f"Hareket kontrol hatası: {e}")
            return False
    
    def stop_movement(self):
        """Araç hareketini durdurur"""
        if self.motor_driver:
            try:
                self.motor_driver.stop()
            except Exception as e:
                print(f"Hareket durdurma hatası: {e}")
    
    def set_pan_tilt_gains(self, pan_kp=0.1, tilt_kp=0.1):
        """Pan-Tilt PID kazançlarını ayarlar"""
        self.pan_kp = pan_kp
        self.tilt_kp = tilt_kp

def main():
    """Test ana fonksiyonu"""
    dart_system = DartRecognition()
    
    try:
        # Sistemi başlat
        if dart_system.start_recognition():
            print("Dart tanıma sistemi çalışıyor...")
            print("Çıkmak için kamera penceresinde 'q' tuşuna basın")
            
            # Ana döngü
            while dart_system.is_running:
                time.sleep(0.1)
                
                # Hedef kilitlenmişse bilgi göster
                if dart_system.is_target_acquired():
                    target_pos = dart_system.get_target_position()
                    target_det = dart_system.get_target_detection()
                    print(f"Hedef kilitlendi: {target_pos}, Güven: {target_det['confidence']:.2f}")
                    
                    # Hedefe hareket (isteğe bağlı)
                    # dart_system.move_towards_target(speed=30)
        
    except KeyboardInterrupt:
        print("\nSistem kapatılıyor...")
    
    finally:
        dart_system.stop_recognition()

if __name__ == "__main__":
    main()
