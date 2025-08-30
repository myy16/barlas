"""
BARLAS Dart Laser Targeting System
Pan-Tilt kamera ile dart tanıma ve lazer hedefleme sistemi
YOLO tabanlı dart detection + servo kontrollü lazer pointer
"""
import cv2
import numpy as np
import time
import threading
import math
from typing import Dict, List, Tuple, Optional
import os
import sys

# BARLAS modüllerini import et
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dart_recognize.yolo_predictions import YOLOPredictions

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    print("RPi.GPIO bulunamadı - simülasyon modu")

class LaserPanTiltController:
    """
    Pan-Tilt servo sistemi ile lazer pointer kontrolü
    Kamera görüntüsündeki dart hedefini lazer ile işaretler
    """
    
    def __init__(self, pan_pin=18, tilt_pin=19, laser_pin=20):
        """
        Lazer Pan-Tilt kontrolcüsü
        
        Args:
            pan_pin: Pan servo GPIO pin (varsayılan: 18)
            tilt_pin: Tilt servo GPIO pin (varsayılan: 19) 
            laser_pin: Lazer GPIO pin (varsayılan: 20)
        """
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        self.laser_pin = laser_pin
        
        # Servo pozisyonları (0-180 derece)
        self.pan_position = 90   # Başlangıç merkez
        self.tilt_position = 90  # Başlangıç merkez
        
        # Servo limitleri
        self.pan_min = 10
        self.pan_max = 170
        self.tilt_min = 30
        self.tilt_max = 150
        
        # Lazer durumu
        self.laser_active = False
        
        # PWM nesneleri
        self.pan_pwm = None
        self.tilt_pwm = None
        
        # Kalibrasyon değerleri (kamera-lazer offset)
        self.calibration_offset_x = 0  # Piksel cinsinden X offset
        self.calibration_offset_y = 0  # Piksel cinsinden Y offset
        
        # GPIO kurulumu
        if RPI_AVAILABLE:
            self.setup_gpio()
        else:
            print("[LaserPanTilt] Simülasyon modu - gerçek donanım yok")
        
        print(f"[LaserPanTilt] Başlatıldı - Pan:{self.pan_position}°, Tilt:{self.tilt_position}°")
    
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
            
            # Başlangıç pozisyonu
            self.move_to_position(self.pan_position, self.tilt_position)
            
            print("[LaserPanTilt] GPIO kurulumu tamamlandı")
            
        except Exception as e:
            print(f"[LaserPanTilt] GPIO kurulum hatası: {e}")
    
    def angle_to_duty_cycle(self, angle):
        """Servo açısını PWM duty cycle'a çevirir"""
        return 2.5 + (angle / 180.0) * 10.0
    
    def move_to_position(self, pan_angle, tilt_angle):
        """Belirtilen açılara hareket eder"""
        
        # Açı sınırlarını kontrol et
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        self.pan_position = pan_angle
        self.tilt_position = tilt_angle
        
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
                print(f"[LaserPanTilt] Servo hareket hatası: {e}")
        
        print(f"[LaserPanTilt] Pozisyon: Pan:{pan_angle:.1f}°, Tilt:{tilt_angle:.1f}°")
    
    def enable_laser(self):
        """Lazer pointer'ı açar"""
        if RPI_AVAILABLE:
            try:
                GPIO.output(self.laser_pin, GPIO.HIGH)
                self.laser_active = True
                print("[LaserPanTilt] 🔴 LAZER AKTİF")
            except Exception as e:
                print(f"[LaserPanTilt] Lazer açma hatası: {e}")
        else:
            self.laser_active = True
            print("[LaserPanTilt] 🔴 LAZER AKTİF (Simülasyon)")
    
    def disable_laser(self):
        """Lazer pointer'ı kapatır"""
        if RPI_AVAILABLE:
            try:
                GPIO.output(self.laser_pin, GPIO.LOW)
                self.laser_active = False
                print("[LazerPanTilt] ⚫ Lazer kapatıldı")
            except Exception as e:
                print(f"[LaserPanTilt] Lazer kapatma hatası: {e}")
        else:
            self.laser_active = False
            print("[LaserPanTilt] ⚫ Lazer kapatıldı (Simülasyon)")
    
    def pixel_to_angle(self, pixel_x, pixel_y, frame_width, frame_height):
        """
        Kamera piksel koordinatlarını servo açılarına çevirir
        
        Args:
            pixel_x, pixel_y: Hedef piksel koordinatları
            frame_width, frame_height: Kamera çözünürlüğü
            
        Returns:
            (pan_angle, tilt_angle): Servo açıları
        """
        
        # Merkez noktasından fark
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # Kalibrasyon offset'i uygula
        offset_x = pixel_x - center_x + self.calibration_offset_x
        offset_y = pixel_y - center_y + self.calibration_offset_y
        
        # Piksel farkını açıya çevir (kamera FOV'a göre)
        # Varsayılan kamera FOV: 60° yatay, 45° dikey
        horizontal_fov = 60
        vertical_fov = 45
        
        pan_adjustment = (offset_x / center_x) * (horizontal_fov / 2)
        tilt_adjustment = -(offset_y / center_y) * (vertical_fov / 2)  # Y ekseni ters
        
        # Mevcut pozisyona ekle
        target_pan = self.pan_position + pan_adjustment
        target_tilt = self.tilt_position + tilt_adjustment
        
        return target_pan, target_tilt
    
    def aim_at_pixel(self, pixel_x, pixel_y, frame_width, frame_height):
        """Belirtilen piksel koordinatına lazer ile nişan alır"""
        
        target_pan, target_tilt = self.pixel_to_angle(pixel_x, pixel_y, frame_width, frame_height)
        
        print(f"[LaserPanTilt] Hedef piksel: ({pixel_x}, {pixel_y})")
        print(f"[LaserPanTilt] Servo hedefi: Pan:{target_pan:.1f}°, Tilt:{target_tilt:.1f}°")
        
        self.move_to_position(target_pan, target_tilt)
        
        # Lazer'i aktif et
        self.enable_laser()
    
    def calibrate_offset(self, pixel_offset_x, pixel_offset_y):
        """Kamera-lazer arasındaki offset'i kalibre eder"""
        self.calibration_offset_x = pixel_offset_x
        self.calibration_offset_y = pixel_offset_y
        print(f"[LaserPanTilt] Kalibrasyon güncellendi: X:{pixel_offset_x}, Y:{pixel_offset_y}")
    
    def center_position(self):
        """Lazer'i merkez pozisyona getirir"""
        self.move_to_position(90, 90)
        self.disable_laser()
    
    def cleanup(self):
        """GPIO temizleme"""
        self.disable_laser()
        
        if RPI_AVAILABLE and self.pan_pwm and self.tilt_pwm:
            try:
                self.pan_pwm.stop()
                self.tilt_pwm.stop()
                GPIO.cleanup()
                print("[LaserPanTilt] GPIO temizlendi")
            except Exception as e:
                print(f"[LaserPanTilt] Temizleme hatası: {e}")


class DartLaserTargetingSystem:
    """
    YOLO dart detection + Pan-Tilt lazer hedefleme sistemi
    Dart'ları otomatik tespit eder ve lazer ile işaretler
    """
    
    def __init__(self, camera_index=0):
        """
        Dart Lazer Hedefleme Sistemi
        
        Args:
            camera_index: Kamera indeksi (varsayılan: 0)
        """
        
        print("[DartLaserSystem] Sistem başlatılıyor...")
        
        # YOLO dart detector
        try:
            self.yolo_detector = YOLOPredictions()
            print("[DartLaserSystem] ✅ YOLO dart detector yüklendi")
        except Exception as e:
            print(f"[DartLaserSystem] ❌ YOLO yükleme hatası: {e}")
            raise
        
        # Lazer Pan-Tilt kontrolcüsü
        try:
            self.laser_pantilt = LaserPanTiltController()
            print("[DartLaserSystem] ✅ Lazer Pan-Tilt sistem hazır")
        except Exception as e:
            print(f"[DartLaserSystem] ❌ Pan-Tilt hatası: {e}")
            raise
        
        # Kamera kurulumu
        self.camera_index = camera_index
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480
        
        # Sistem durumu
        self.is_running = False
        self.is_targeting = False
        self.current_target = None
        self.target_lock_duration = 0.0
        
        # Threading
        self.targeting_thread = None
        
        # Hedefleme parametreleri
        self.target_confidence_threshold = 0.6  # Minimum dart güveni
        self.target_lock_time = 2.0  # Hedefe kilitlenme süresi (saniye)
        self.laser_pulse_duration = 5.0  # Lazer açık kalma süresi
        
        print("[DartLaserSystem] 🎯 Sistem hazır - hedefleme başlayabilir")
    
    def initialize_camera(self):
        """Kamerayı başlatır"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"[DartLaserSystem] ❌ Kamera {self.camera_index} açılamadı!")
                return False
            
            # Kamera ayarları
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Gerçek çözünürlüğü al
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[DartLaserSystem] ✅ Kamera: {self.frame_width}x{self.frame_height}")
            return True
            
        except Exception as e:
            print(f"[DartLaserSystem] Kamera başlatma hatası: {e}")
            return False
    
    def start_targeting_system(self):
        """Dart hedefleme sistemini başlatır"""
        
        if not self.initialize_camera():
            return False
        
        self.is_running = True
        self.targeting_thread = threading.Thread(target=self._targeting_loop, daemon=True)
        self.targeting_thread.start()
        
        print("[DartLaserSystem] 🚀 Hedefleme sistemi başlatıldı")
        return True
    
    def stop_targeting_system(self):
        """Dart hedefleme sistemini durdurur"""
        
        print("[DartLaserSystem] Sistem durduruluyor...")
        
        self.is_running = False
        self.is_targeting = False
        
        if self.targeting_thread:
            self.targeting_thread.join(timeout=3.0)
        
        if self.cap:
            self.cap.release()
        
        # Lazer'i kapat ve merkeze getir
        self.laser_pantilt.center_position()
        
        cv2.destroyAllWindows()
        print("[DartLaserSystem] ⚫ Sistem durduruldu")
    
    def _targeting_loop(self):
        """Ana hedefleme döngüsü (thread'de çalışır)"""
        
        print("[DartLaserSystem] 👁️ Hedefleme döngüsü başladı")
        
        last_target_time = 0
        laser_end_time = 0
        
        while self.is_running and self.cap and self.cap.isOpened():
            
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            try:
                # Dart tespiti yap
                detections = self.yolo_detector.get_detections(frame)
                
                # Güvenli dart'ları filtrele
                valid_darts = [d for d in detections if d['confidence'] >= self.target_confidence_threshold]
                
                current_time = time.time()
                
                if valid_darts:
                    # En büyük ve en güvenli dart'ı seç
                    best_dart = max(valid_darts, key=lambda d: d['confidence'] * (d['bbox'][2] * d['bbox'][3]))
                    
                    x, y, w, h = best_dart['bbox']
                    target_x = x + w // 2
                    target_y = y + h // 2
                    confidence = best_dart['confidence']
                    
                    # Hedef stabil mi kontrol et
                    if self.current_target is None:
                        self.current_target = (target_x, target_y)
                        last_target_time = current_time
                        self.target_lock_duration = 0
                        
                        print(f"[DartLaserSystem] 🎯 YENİ HEDEF: ({target_x}, {target_y}), Güven: {confidence:.2f}")
                    
                    else:
                        # Hedef kararlılığını kontrol et
                        prev_x, prev_y = self.current_target
                        distance = math.sqrt((target_x - prev_x)**2 + (target_y - prev_y)**2)
                        
                        if distance < 30:  # 30 piksel toleransı
                            # Hedef stabil
                            self.target_lock_duration = current_time - last_target_time
                            
                            # Yeterli süre kilitlenmişse lazer ateşle
                            if (self.target_lock_duration >= self.target_lock_time and 
                                not self.is_targeting and 
                                current_time > laser_end_time):
                                
                                self._fire_laser_at_target(target_x, target_y)
                                laser_end_time = current_time + self.laser_pulse_duration
                        
                        else:
                            # Hedef değişti, yeniden başla
                            self.current_target = (target_x, target_y)
                            last_target_time = current_time
                            self.target_lock_duration = 0
                            
                            print(f"[DartLaserSystem] 🔄 Hedef değişti: ({target_x}, {target_y})")
                
                else:
                    # Dart bulunamadı
                    if self.current_target is not None:
                        print("[DartLaserSystem] ❌ Hedef kaybedildi")
                        self.current_target = None
                        self.target_lock_duration = 0
                        
                        # Lazer'i kapat
                        if self.is_targeting:
                            self.laser_pantilt.disable_laser()
                            self.is_targeting = False
                
                # Lazer süresini kontrol et
                if self.is_targeting and current_time > laser_end_time:
                    self.laser_pantilt.disable_laser()
                    self.is_targeting = False
                    print("[DartLaserSystem] ⏰ Lazer süresi doldu")
                
                # Görsel çıktı
                display_frame = self._draw_targeting_info(frame, valid_darts)
                cv2.imshow('BARLAS Dart Laser Targeting', display_frame)
                
                # Çıkış kontrolü
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.is_running = False
                    break
                
            except Exception as e:
                print(f"[DartLaserSystem] Döngü hatası: {e}")
                time.sleep(0.1)
            
            time.sleep(0.033)  # ~30 FPS
    
    def _fire_laser_at_target(self, target_x, target_y):
        """Belirtilen hedefe lazer ateşler"""
        
        print(f"[DartLaserSystem] 🔥 LAZER ATEŞİ: ({target_x}, {target_y})")
        
        try:
            # Lazer'i hedefe yönlendir
            self.laser_pantilt.aim_at_pixel(target_x, target_y, self.frame_width, self.frame_height)
            
            self.is_targeting = True
            
            print("[DartLaserSystem] 🎯 HEDEF KİLİTLENDİ - LAZER AKTİF")
            
        except Exception as e:
            print(f"[DartLaserSystem] Lazer atış hatası: {e}")
    
    def _draw_targeting_info(self, frame, detections):
        """Hedefleme bilgilerini frame üzerine çizer"""
        
        # Dart tespitlerini çiz
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            
            # Bounding box
            color = (0, 255, 0) if confidence >= self.target_confidence_threshold else (0, 165, 255)
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            
            # Güven skoru
            cv2.putText(frame, f"Dart: {confidence:.2f}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Mevcut hedef
        if self.current_target:
            target_x, target_y = self.current_target
            
            # Hedef merkezi
            cv2.circle(frame, (target_x, target_y), 15, (255, 255, 0), 3)
            cv2.circle(frame, (target_x, target_y), 5, (255, 255, 0), -1)
            
            # Kilitlenme çizgisi
            if self.target_lock_duration > 0:
                lock_progress = min(self.target_lock_duration / self.target_lock_time, 1.0)
                bar_width = int(100 * lock_progress)
                
                cv2.rectangle(frame, (target_x - 50, target_y - 30), 
                             (target_x - 50 + bar_width, target_y - 25), (0, 255, 255), -1)
                cv2.rectangle(frame, (target_x - 50, target_y - 30), 
                             (target_x + 50, target_y - 25), (255, 255, 255), 2)
                
                # Kilitlenme metni
                if lock_progress >= 1.0:
                    cv2.putText(frame, "LOCKED", (target_x - 30, target_y - 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Sistem durumu
        status_color = (0, 255, 0) if self.is_targeting else (255, 255, 255)
        laser_status = "LAZER AKTİF" if self.is_targeting else "HEDEF ARANIYOṘ"
        
        cv2.putText(frame, laser_status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Frame merkezi
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        cv2.circle(frame, (center_x, center_y), 3, (255, 0, 0), -1)
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 1)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 1)
        
        return frame
    
    def set_targeting_parameters(self, confidence_threshold=0.6, lock_time=2.0, laser_duration=5.0):
        """Hedefleme parametrelerini ayarlar"""
        self.target_confidence_threshold = confidence_threshold
        self.target_lock_time = lock_time
        self.laser_pulse_duration = laser_duration
        
        print(f"[DartLaserSystem] Parametreler güncellendi:")
        print(f"  Güven eşiği: {confidence_threshold}")
        print(f"  Kilitlenme süresi: {lock_time}s")
        print(f"  Lazer süresi: {laser_duration}s")
    
    def manual_laser_control(self, enable=True):
        """Manuel lazer kontrolü"""
        if enable:
            self.laser_pantilt.enable_laser()
        else:
            self.laser_pantilt.disable_laser()
    
    def calibrate_laser_offset(self, pixel_x_offset, pixel_y_offset):
        """Lazer-kamera kalibrasyonu"""
        self.laser_pantilt.calibrate_offset(pixel_x_offset, pixel_y_offset)
    
    def cleanup(self):
        """Sistem temizleme"""
        self.stop_targeting_system()
        self.laser_pantilt.cleanup()
        print("[DartLaserSystem] Sistem temizliği tamamlandı")


def main():
    """Ana test fonksiyonu"""
    
    print("=" * 60)
    print("🎯 BARLAS DART LASER TARGETING SYSTEM 🎯")
    print("=" * 60)
    
    try:
        # Dart lazer sistemi oluştur
        dart_laser = DartLaserTargetingSystem(camera_index=0)
        
        print("\n🚀 Sistem başlatılıyor...")
        
        # Hedefleme parametreleri ayarla
        dart_laser.set_targeting_parameters(
            confidence_threshold=0.5,  # Dart güven eşiği
            lock_time=1.5,            # Kilitlenme süresi
            laser_duration=3.0        # Lazer açık kalma süresi
        )
        
        # Sistemi başlat
        if dart_laser.start_targeting_system():
            print("✅ Hedefleme sistemi aktif!")
            print("\n📋 Kontroller:")
            print("  'q' - Çıkış")
            print("  Otomatik dart tespit ve lazer hedefleme aktif")
            print("\n🎯 Dart aramamaya başlıyor...")
            
            # Ana döngü - sistem çalışırken bekle
            try:
                while dart_laser.is_running:
                    time.sleep(0.1)
                    
                    # Ek kontroller burada eklenebilir
                    # Örnek: klavye girişi, web arayüzü vs.
                    
            except KeyboardInterrupt:
                print("\n⚠️ Kullanıcı tarafından iptal edildi")
        
        else:
            print("❌ Sistem başlatılamadı!")
    
    except Exception as e:
        print(f"❌ Sistem hatası: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if 'dart_laser' in locals():
            dart_laser.cleanup()
        
        print("\n🏁 Program sonlandırıldı")


if __name__ == "__main__":
    main()
