"""
BARLAS Dart Laser Targeting System
Ana hedefleme sistemi - tüm modülleri entegre eder
"""
import cv2
import numpy as np
import time
import threading
import math
from typing import Dict, List, Tuple, Optional

from dart_detector import DartDetector
from laser_controller import LaserPanTiltController


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
        
        # Dart detector
        try:
            self.dart_detector = DartDetector(confidence_threshold=0.6)
            print("[DartLaserSystem] ✅ Dart detector hazır")
        except Exception as e:
            print(f"[DartLaserSystem] ❌ Dart detector hatası: {e}")
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
        self.stability_threshold = 30  # Piksel kararlılık eşiği
        
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
        previous_center = None
        
        while self.is_running and self.cap and self.cap.isOpened():
            
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            try:
                # Dart tespiti yap
                detections = self.dart_detector.detect_darts(frame)
                
                current_time = time.time()
                
                if detections:
                    # Kararlı dart'ı seç
                    stable_dart = self.dart_detector.get_stable_dart(
                        detections, previous_center, self.stability_threshold)
                    
                    if stable_dart:
                        target_x, target_y = stable_dart['center']
                        confidence = stable_dart['confidence']
                        
                        # Hedef stabil mi kontrol et
                        if self.current_target is None:
                            self.current_target = (target_x, target_y)
                            previous_center = (target_x, target_y)
                            last_target_time = current_time
                            self.target_lock_duration = 0
                            
                            print(f"[DartLaserSystem] 🎯 YENİ HEDEF: ({target_x}, {target_y}), Güven: {confidence:.2f}")
                        
                        else:
                            # Hedef kararlılığını kontrol et
                            prev_x, prev_y = self.current_target
                            distance = math.sqrt((target_x - prev_x)**2 + (target_y - prev_y)**2)
                            
                            if distance < self.stability_threshold:
                                # Hedef stabil
                                self.target_lock_duration = current_time - last_target_time
                                previous_center = (target_x, target_y)
                                
                                # Yeterli süre kilitlenmişse lazer ateşle
                                if (self.target_lock_duration >= self.target_lock_time and 
                                    not self.is_targeting and 
                                    current_time > laser_end_time):
                                    
                                    self._fire_laser_at_target(target_x, target_y)
                                    laser_end_time = current_time + self.laser_pulse_duration
                            
                            else:
                                # Hedef değişti, yeniden başla
                                self.current_target = (target_x, target_y)
                                previous_center = (target_x, target_y)
                                last_target_time = current_time
                                self.target_lock_duration = 0
                                
                                print(f"[DartLaserSystem] 🔄 Hedef değişti: ({target_x}, {target_y})")
                
                else:
                    # Dart bulunamadı
                    if self.current_target is not None:
                        print("[DartLaserSystem] ❌ Hedef kaybedildi")
                        self.current_target = None
                        previous_center = None
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
                display_frame = self._draw_targeting_info(frame, detections)
                cv2.imshow('BARLAS Dart Laser Targeting', display_frame)
                
                # Çıkış kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.is_running = False
                    break
                elif key == ord(' '):
                    # Manuel lazer kontrolü
                    if self.laser_pantilt.laser_active:
                        self.laser_pantilt.disable_laser()
                    else:
                        self.laser_pantilt.enable_laser()
                elif key == ord('c'):
                    # Merkeze getir
                    self.laser_pantilt.center_position()
                
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
        highlight_dart = None
        if self.current_target and detections:
            # Mevcut hedefe en yakın dart'ı vurgula
            tx, ty = self.current_target
            min_dist = float('inf')
            
            for detection in detections:
                dx, dy = detection['center']
                dist = math.sqrt((dx - tx)**2 + (dy - ty)**2)
                if dist < min_dist:
                    min_dist = dist
                    highlight_dart = detection
        
        display_frame = self.dart_detector.draw_detections(frame, detections, highlight_dart)
        
        # Mevcut hedef
        if self.current_target:
            target_x, target_y = self.current_target
            
            # Hedef merkezi
            cv2.circle(display_frame, (target_x, target_y), 15, (255, 255, 0), 3)
            cv2.circle(display_frame, (target_x, target_y), 5, (255, 255, 0), -1)
            
            # Kilitlenme çizgisi
            if self.target_lock_duration > 0:
                lock_progress = min(self.target_lock_duration / self.target_lock_time, 1.0)
                bar_width = int(100 * lock_progress)
                
                cv2.rectangle(display_frame, (target_x - 50, target_y - 30), 
                             (target_x - 50 + bar_width, target_y - 25), (0, 255, 255), -1)
                cv2.rectangle(display_frame, (target_x - 50, target_y - 30), 
                             (target_x + 50, target_y - 25), (255, 255, 255), 2)
                
                # Kilitlenme metni
                if lock_progress >= 1.0:
                    cv2.putText(display_frame, "LOCKED", (target_x - 30, target_y - 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, f"LOCKING {lock_progress*100:.0f}%", 
                               (target_x - 50, target_y - 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Sistem durumu
        status_color = (0, 255, 0) if self.is_targeting else (255, 255, 255)
        laser_status = "LAZER AKTİF" if self.is_targeting else "HEDEF ARANIYOṘ"
        
        cv2.putText(display_frame, laser_status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Sistem parametreleri
        params = [
            f"Confidence: {self.target_confidence_threshold}",
            f"Lock Time: {self.target_lock_time}s",
            f"Laser Duration: {self.laser_pulse_duration}s"
        ]
        
        for i, param in enumerate(params):
            cv2.putText(display_frame, param, (10, display_frame.shape[0] - 60 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Frame merkezi
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        cv2.circle(display_frame, (center_x, center_y), 3, (255, 0, 0), -1)
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 1)
        
        # Kontrol bilgileri
        controls = ["'q': Quit", "'space': Toggle Laser", "'c': Center"]
        for i, control in enumerate(controls):
            cv2.putText(display_frame, control, (display_frame.shape[1] - 200, 30 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return display_frame
    
    def set_targeting_parameters(self, confidence_threshold=0.6, lock_time=2.0, 
                               laser_duration=5.0, stability_threshold=30):
        """Hedefleme parametrelerini ayarlar"""
        self.target_confidence_threshold = confidence_threshold
        self.target_lock_time = lock_time
        self.laser_pulse_duration = laser_duration
        self.stability_threshold = stability_threshold
        
        # Dart detector'ı da güncelle
        self.dart_detector.set_confidence_threshold(confidence_threshold)
        
        print(f"[DartLaserSystem] Parametreler güncellendi:")
        print(f"  Güven eşiği: {confidence_threshold}")
        print(f"  Kilitlenme süresi: {lock_time}s")
        print(f"  Lazer süresi: {laser_duration}s")
        print(f"  Kararlılık eşiği: {stability_threshold}px")
    
    def manual_laser_control(self, enable=True):
        """Manuel lazer kontrolü"""
        if enable:
            self.laser_pantilt.enable_laser()
        else:
            self.laser_pantilt.disable_laser()
    
    def calibrate_laser_offset(self, pixel_x_offset, pixel_y_offset):
        """Lazer-kamera kalibrasyonu"""
        self.laser_pantilt.calibrate_offset(pixel_x_offset, pixel_y_offset)
    
    def get_system_status(self) -> Dict:
        """Sistem durumunu döndürür"""
        
        detector_stats = self.dart_detector.get_detection_stats()
        
        return {
            'is_running': self.is_running,
            'is_targeting': self.is_targeting,
            'current_target': self.current_target,
            'target_lock_duration': self.target_lock_duration,
            'laser_active': self.laser_pantilt.laser_active,
            'pan_position': self.laser_pantilt.pan_position,
            'tilt_position': self.laser_pantilt.tilt_position,
            'detector_stats': detector_stats,
            'camera_resolution': (self.frame_width, self.frame_height)
        }
    
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
            laser_duration=3.0,       # Lazer açık kalma süresi
            stability_threshold=25    # Kararlılık eşiği
        )
        
        # Sistemi başlat
        if dart_laser.start_targeting_system():
            print("✅ Hedefleme sistemi aktif!")
            print("\n📋 Kontroller:")
            print("  'q' - Çıkış")
            print("  'space' - Manuel lazer açma/kapama")
            print("  'c' - Merkeze getir")
            print("  Otomatik dart tespit ve lazer hedefleme aktif")
            print("\n🎯 Dart aramamaya başlıyor...")
            
            # Ana döngü - sistem çalışırken bekle
            try:
                while dart_laser.is_running:
                    time.sleep(0.1)
                    
                    # Sistem durumunu periyodik olarak yazdır
                    if int(time.time()) % 10 == 0:
                        status = dart_laser.get_system_status()
                        print(f"\n📊 Sistem Durumu:")
                        print(f"  Hedefleme: {'AKTİF' if status['is_targeting'] else 'BEKLEMEDE'}")
                        print(f"  Mevcut Hedef: {status['current_target']}")
                        print(f"  Lazer: {'AKTİF' if status['laser_active'] else 'KAPALI'}")
                        print(f"  Servo: Pan={status['pan_position']:.1f}°, Tilt={status['tilt_position']:.1f}°")
                        time.sleep(1)  # Tekrar yazdırmayı engelle
                    
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
