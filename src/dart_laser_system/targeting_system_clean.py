"""
BARLAS Dart Laser Targeting System - Eski Sistem
Ana hedefleme sistemi - tum modulleri entegre eder
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
    Dartlari otomatik tespit eder ve lazer ile isaretler
    """
    
    def __init__(self, camera_index=0):
        """
        Dart Lazer Hedefleme Sistemi
        
        Args:
            camera_index: Kamera indeksi (varsayilan: 0)
        """
        
        print("[DartLaserSystem] Sistem baslatiliyor...")
        
        # Dart detector
        try:
            self.dart_detector = DartDetector(confidence_threshold=0.6)
            print("[DartLaserSystem] ✅ Dart detector hazir")
        except Exception as e:
            print(f"[DartLaserSystem] ❌ Dart detector hatasi: {e}")
            raise
        
        # Lazer Pan-Tilt kontrolcusu
        try:
            self.laser_pantilt = LaserPanTiltController()
            print("[DartLaserSystem] ✅ Lazer Pan-Tilt sistem hazir")
        except Exception as e:
            print(f"[DartLaserSystem] ❌ Pan-Tilt hatasi: {e}")
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
        self.target_confidence_threshold = 0.6  # Minimum dart guveni
        self.target_lock_time = 2.0  # Hedefe kilitlenme suresi (saniye)
        self.laser_pulse_duration = 5.0  # Lazer acik kalma suresi
        self.stability_threshold = 30  # Piksel kararlilik esigi
        
        print("[DartLaserSystem] 🎯 Sistem hazir - hedefleme baslayabilir")
    
    def initialize_camera(self):
        """Kamerayi baslatir"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"[DartLaserSystem] ❌ Kamera {self.camera_index} acilamadi!")
                return False
            
            # Kamera ayarlari
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Gercek cozunurlugu al
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[DartLaserSystem] ✅ Kamera: {self.frame_width}x{self.frame_height}")
            return True
            
        except Exception as e:
            print(f"[DartLaserSystem] Kamera baslatma hatasi: {e}")
            return False
    
    def start_targeting_system(self):
        """Dart hedefleme sistemini baslatir"""
        
        if not self.initialize_camera():
            return False
        
        self.is_running = True
        self.targeting_thread = threading.Thread(target=self._targeting_loop, daemon=True)
        self.targeting_thread.start()
        
        print("[DartLaserSystem] 🚀 Hedefleme sistemi baslatildi")
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
        
        # Lazeri kapat ve merkeze getir
        self.laser_pantilt.center_position()
        
        cv2.destroyAllWindows()
        print("[DartLaserSystem] ⚫ Sistem durduruldu")
    
    def _targeting_loop(self):
        """Ana hedefleme dongusu (threadde calisir)"""
        
        print("[DartLaserSystem] 🎯⚡ Hedefleme dongusu basladi")
        
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
                    # Kararli dartı sec
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
                            
                            print(f"[DartLaserSystem] 🎯 YENI HEDEF: ({target_x}, {target_y}), Guven: {confidence:.2f}")
                        
                        else:
                            # Hedef kararliligini kontrol et
                            prev_x, prev_y = self.current_target
                            distance = math.sqrt((target_x - prev_x)**2 + (target_y - prev_y)**2)
                            
                            if distance < self.stability_threshold:
                                # Hedef stabil
                                self.target_lock_duration = current_time - last_target_time
                                previous_center = (target_x, target_y)
                                
                                print(f"[DartLaserSystem] 📍 Hedef stabil - Kilit suresi: {self.target_lock_duration:.1f}s")
                                
                                # Pan-tilt pozisyonu goster
                                try:
                                    pan_pos, tilt_pos = self.laser_pantilt.get_current_position()
                                    print(f"[DartLaserSystem] 🧭 Pan: {pan_pos:.1f}°, Tilt: {tilt_pos:.1f}°")
                                except:
                                    print("[DartLaserSystem] 🧭 Pan-Tilt pozisyon alinamadi")
                                
                                # Yeterli sure kilitlenmisse lazer atesle
                                if (self.target_lock_duration >= self.target_lock_time and 
                                    not self.is_targeting and 
                                    current_time > laser_end_time):
                                    
                                    self._fire_laser_at_target(target_x, target_y)
                                    laser_end_time = current_time + self.laser_pulse_duration
                            
                            else:
                                # Hedef degisti, yeniden basla
                                self.current_target = (target_x, target_y)
                                previous_center = (target_x, target_y)
                                last_target_time = current_time
                                self.target_lock_duration = 0
                                
                                print(f"[DartLaserSystem] 🔄 Hedef degisti: ({target_x}, {target_y})")
                
                else:
                    # Dart bulunamadi
                    if self.current_target is not None:
                        print("[DartLaserSystem] ❌ Hedef kaybedildi")
                        self.current_target = None
                        previous_center = None
                        self.target_lock_duration = 0
                        
                        # Lazeri kapat
                        if self.is_targeting:
                            self.laser_pantilt.disable_laser()
                            self.is_targeting = False
                
                # Lazer suresini kontrol et
                if self.is_targeting and current_time > laser_end_time:
                    self.laser_pantilt.disable_laser()
                    self.is_targeting = False
                    print("[DartLaserSystem] 🔚 Lazer suresi doldu")
                
                # Gorsel cikti
                display_frame = self._draw_targeting_info(frame, detections)
                cv2.imshow('BARLAS Dart Laser Targeting', display_frame)
                
                # Cikis kontrolu
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.is_running = False
                    break
                elif key == ord(' '):
                    # Manuel lazer kontrolu
                    try:
                        if self.laser_pantilt.laser_active:
                            self.laser_pantilt.disable_laser()
                        else:
                            self.laser_pantilt.enable_laser()
                    except:
                        print("[DartLaserSystem] Manuel lazer kontrolu basarisiz")
                elif key == ord('c'):
                    # Merkeze getir
                    self.laser_pantilt.center_position()
                
            except Exception as e:
                print(f"[DartLaserSystem] Dongu hatasi: {e}")
                time.sleep(0.1)
            
            time.sleep(0.033)  # ~30 FPS
    
    def _fire_laser_at_target(self, target_x, target_y):
        """Belirtilen hedefe lazer atesler"""
        
        print(f"[DartLaserSystem] 🔥 LAZER ATESI: ({target_x}, {target_y})")
        
        try:
            # Lazeri hedefe yonlendir
            self.laser_pantilt.aim_at_pixel(target_x, target_y, self.frame_width, self.frame_height)
            
            # Pozisyon bilgisi
            try:
                pan_pos, tilt_pos = self.laser_pantilt.get_current_position()
                print(f"[DartLaserSystem] 🎯 Hedef pozisyon - Pan: {pan_pos:.1f}°, Tilt: {tilt_pos:.1f}°")
            except:
                print("[DartLaserSystem] 🎯 Hedef pozisyon bilgisi alinamadi")
            
            self.is_targeting = True
            
            print("[DartLaserSystem] 🎯 HEDEF KILITLENDI - LAZER AKTIF")
            
        except Exception as e:
            print(f"[DartLaserSystem] Lazer atesi hatasi: {e}")
    
    def _draw_targeting_info(self, frame, detections):
        """Frame uzerine hedefleme bilgilerini cizer"""
        
        display_frame = frame.copy()
        
        # Dart tespitlerini ciz
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            center = detection['center']
            
            # Bbox
            color = (0, 255, 0) if confidence >= self.target_confidence_threshold else (0, 255, 255)
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, 2)
            
            # Merkez
            cv2.circle(display_frame, center, 5, color, -1)
            
            # Guven degeri
            cv2.putText(display_frame, f"{confidence:.2f}", 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Mevcut hedefi vurgula
        if self.current_target:
            tx, ty = self.current_target
            cv2.circle(display_frame, (tx, ty), 25, (255, 255, 0), 3)
            cv2.putText(display_frame, "TARGET LOCKED", (tx-60, ty-35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Sistem durumu bilgisi
        status_y = 30
        cv2.putText(display_frame, f"Hedefleme: {'AKTIF' if self.is_targeting else 'BEKLEMEDE'}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.putText(display_frame, f"Mevcut Hedef: {self.current_target}", 
                   (10, status_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.putText(display_frame, f"Lazer: {'ACIK' if self.is_targeting else 'KAPALI'}", 
                   (10, status_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Pan-tilt pozisyonu
        try:
            pan_pos, tilt_pos = self.laser_pantilt.get_current_position()
            cv2.putText(display_frame, f"Servo: Pan={pan_pos:.1f}, Tilt={tilt_pos:.1f}", 
                       (10, status_y + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        except:
            cv2.putText(display_frame, "Servo: Pan=?, Tilt=?", 
                       (10, status_y + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
        
        # Kilit suresi
        if self.target_lock_duration > 0:
            cv2.putText(display_frame, f"Kilit suresi: {self.target_lock_duration:.1f}s", 
                       (10, status_y + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Cerceve merkezi
        h, w = display_frame.shape[:2]
        cv2.circle(display_frame, (w//2, h//2), 3, (255, 0, 0), -1)
        cv2.line(display_frame, (w//2-20, h//2), (w//2+20, h//2), (255, 0, 0), 1)
        cv2.line(display_frame, (w//2, h//2-20), (w//2, h//2+20), (255, 0, 0), 1)
        
        return display_frame

    def get_system_status(self):
        """Sistem durumu bilgilerini dondurur"""
        return {
            'is_running': self.is_running,
            'is_targeting': self.is_targeting,
            'current_target': self.current_target,
            'target_lock_duration': self.target_lock_duration,
            'camera_resolution': (self.frame_width, self.frame_height)
        }


def main():
    """Ana test fonksiyonu"""
    print("🎯 BARLAS DART LASER TARGETING SYSTEM")
    print("=" * 60)
    
    try:
        # Sistem olustur
        targeting_system = DartLaserTargetingSystem(camera_index=0)
        
        print("\n🎮 Kontroller:")
        print("  'q': Sistem durdur")
        print("  ' ': Manuel lazer on/off")
        print("  'c': Pan-tilt merkeze getir")
        
        # Sistemi baslat
        if targeting_system.start_targeting_system():
            print("\n✅ Sistem basariyla baslatildi!")
            print("🎯 Dart arama ve hedefleme aktif...")
            
            # Ana loop (threadler calisiyor)
            try:
                while targeting_system.is_running:
                    time.sleep(3)
                    
                    # Durum bilgisi
                    status = targeting_system.get_system_status()
                    print(f"📊 Sistem Durumu:")
                    print(f"   Hedefleme: {'AKTIF' if status['is_targeting'] else 'BEKLEMEDE'}")
                    print(f"   Mevcut Hedef: {status['current_target']}")
                    print(f"   Lazer: {'ACIK' if status['is_targeting'] else 'KAPALI'}")
                    
                    # Pan-tilt pozisyonu
                    try:
                        pan_pos, tilt_pos = targeting_system.laser_pantilt.get_current_position()
                        print(f"   Servo: Pan={pan_pos:.1f}°, Tilt={tilt_pos:.1f}°")
                    except:
                        print("   Servo: Pozisyon alinamadi")
                        
            except KeyboardInterrupt:
                print("\n⚠️ Kullanici tarafindan iptal edildi")
        else:
            print("❌ Sistem baslatilamadi!")
    
    except KeyboardInterrupt:
        print("\n⚠️ Kullanici tarafindan iptal edildi")
    except Exception as e:
        print(f"❌ Sistem hatasi: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if 'targeting_system' in locals():
            targeting_system.stop_targeting_system()
            print("[DartLaserSystem] Sistem temizligi tamamlandi")
        
        print("\n🏁 Program sonlandirildi")


if __name__ == "__main__":
    main()
