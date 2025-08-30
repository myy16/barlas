"""
BARLAS Dart Laser Targeting System - Gelişmiş Entegrasyonlu Versiyon
Arduino Simulator ile çalışabilir, optimize edilmiş parametreler
"""
import cv2
import numpy as np
import time
import threading
import math
from typing import Dict, List, Tuple, Optional

from dart_detector import DartDetector
try:
    from arduino_controller_fixed import ArduinoPanTiltController
    ARDUINO_AVAILABLE = True
except ImportError:
    ARDUINO_AVAILABLE = False

# Arduino yoksa simulator kullan
if not ARDUINO_AVAILABLE:
    from arduino_simulator import ArduinoSimulator as ArduinoPanTiltController
    print("[DartLaserSystem] ⚠️ Arduino bulunamadı, simülatör kullanılacak")


class DartLaserTargetingSystem:
    """
    YOLO dart detection + Pan-Tilt lazer hedefleme sistemi
    Arduino ile gerçek veya simülatör ile çalışır
    """
    
    def __init__(self, camera_index=0, arduino_port=None):
        """
        Dart Lazer Hedefleme Sistemi
        
        Args:
            camera_index: Kamera indeksi (varsayılan: 0)
            arduino_port: Arduino portu (None = otomatik tespit veya simulatör)
        """
        
        print("[DartLaserSystem] 🚀 Sistem başlatılıyor...")
        
        # Dart detector - optimize edilmiş parametrelerle
        try:
            self.dart_detector = DartDetector(confidence_threshold=0.5)
            
            # Hough Circle parametrelerini optimize et
            self.dart_detector.set_hough_params(
                param1=60,           # Canny edge threshold
                param2=20,           # Accumulator threshold (daha hassas)
                minDist_ratio=0.25,  # Daha yakın circle'lara izin ver
                minRadius_ratio=0.1, # Daha küçük dart'lara izin ver
                blur_kernel=7,       # Daha az blur
                blur_sigma=1.5       # Daha az blur
            )
            
            print("[DartLaserSystem] ✅ Dart detector hazır (optimize edilmiş)")
        except Exception as e:
            print(f"[DartLaserSystem] ❌ Dart detector hatası: {e}")
            raise
        
        # Arduino Pan-Tilt kontrolcüsü
        try:
            if ARDUINO_AVAILABLE and arduino_port:
                self.arduino_controller = ArduinoPanTiltController(arduino_port)
                print("[DartLaserSystem] ✅ Gerçek Arduino pan-tilt sistem hazır")
            else:
                self.arduino_controller = ArduinoPanTiltController()
                print("[DartLaserSystem] ✅ Arduino simülatör hazır")
                
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
        self.last_target_time = 0
        
        # Threading
        self.targeting_thread = None
        self.display_thread = None
        
        # Hedefleme parametreleri - optimize edilmiş
        self.target_confidence_threshold = 0.5  # Minimum dart güveni
        self.target_lock_time = 1.5             # Hedefe kilitlenme süresi (azaltıldı)
        self.laser_pulse_duration = 3.0         # Lazer açık kalma süresi
        self.stability_threshold = 40           # Piksel kararlılık eşiği (artırıldı)
        
        # Kamera-servo kalibrasyonu
        self.calibration = {
            'center_x': self.frame_width // 2,
            'center_y': self.frame_height // 2,
            'pan_scale': 0.15,     # piksel -> derece dönüşüm oranı
            'tilt_scale': 0.12,
            'pan_offset': 0,       # kalibrasyon offseti
            'tilt_offset': 0
        }
        
        # İstatistikler
        self.stats = {
            'targets_detected': 0,
            'targets_locked': 0,
            'laser_fires': 0,
            'accuracy': 0.0
        }
        
        print("[DartLaserSystem] 🎯 Sistem hazır - hedefleme başlayabilir")
    
    def initialize_camera(self) -> bool:
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
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Buffer'ı küçült (gecikme azalır)
            
            # Gerçek çözünürlüğü al
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            # Kalibrasyonu güncelle
            self.calibration['center_x'] = self.frame_width // 2
            self.calibration['center_y'] = self.frame_height // 2
            
            print(f"[DartLaserSystem] ✅ Kamera: {self.frame_width}x{self.frame_height}")
            return True
            
        except Exception as e:
            print(f"[DartLaserSystem] Kamera başlatma hatası: {e}")
            return False
    
    def pixel_to_servo_angle(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """Piksel koordinatlarını servo açılarına dönüştürür"""
        
        # Merkeze göre göreceli pozisyon
        delta_x = pixel_x - self.calibration['center_x']
        delta_y = pixel_y - self.calibration['center_y']
        
        # Servo açılarını hesapla
        current_pan, current_tilt = self.arduino_controller.get_position()
        
        # Pan hareketi (X ekseni) - ters yönlü
        target_pan = current_pan - (delta_x * self.calibration['pan_scale'])
        target_pan += self.calibration['pan_offset']
        
        # Tilt hareketi (Y ekseni)
        target_tilt = current_tilt + (delta_y * self.calibration['tilt_scale'])
        target_tilt += self.calibration['tilt_offset']
        
        # Limitleri uygula
        target_pan = max(0, min(180, target_pan))
        target_tilt = max(20, min(160, target_tilt))
        
        return target_pan, target_tilt
    
    def is_target_stable(self, current_center: Tuple[int, int], 
                        previous_center: Optional[Tuple[int, int]] = None) -> bool:
        """Hedefin kararlı olup olmadığını kontrol eder"""
        
        if previous_center is None:
            return True
        
        # Mesafeyi hesapla
        distance = math.sqrt(
            (current_center[0] - previous_center[0])**2 + 
            (current_center[1] - previous_center[1])**2
        )
        
        return distance < self.stability_threshold
    
    def track_and_fire(self, dart_info: Dict) -> bool:
        """Dart'ı takip et ve lazer ateşle"""
        
        center_x, center_y = dart_info['center']
        confidence = dart_info['confidence']
        
        print(f"[DartLaserSystem] 🎯 Hedef kilitlendi: ({center_x}, {center_y}) Güven: {confidence:.2f}")
        
        # Servo açılarını hesapla
        target_pan, target_tilt = self.pixel_to_servo_angle(center_x, center_y)
        
        print(f"[DartLaserSystem] 🔄 Servo hedefleniyor: Pan={target_pan:.1f}°, Tilt={target_tilt:.1f}°")
        
        # Servo'yu hareket ettir
        success = self.arduino_controller.move_to_position(target_pan, target_tilt, smooth=True)
        
        if success:
            # Hareket tamamlandıktan sonra lazer ateşle
            time.sleep(0.3)  # Servo'nun sabitlenmesi için bekle
            
            print(f"[DartLaserSystem] 🔴 Lazer ateşleniyor ({self.laser_pulse_duration}s)...")
            self.arduino_controller.fire_laser(self.laser_pulse_duration)
            
            # İstatistikleri güncelle
            self.stats['laser_fires'] += 1
            
            return True
        
        return False
    
    def start_targeting(self):
        """Ana hedefleme döngüsünü başlatır"""
        
        if not self.initialize_camera():
            return
        
        self.is_running = True
        print("[DartLaserSystem] 🎯 Hedefleme sistemi başlatıldı!")
        print("Kontroller: ESC=Çıkış, SPACE=Manuel ateş, C=Kalibrasyon")
        
        # Arduino'yu merkeze getir
        self.arduino_controller.center_position()
        time.sleep(1)
        
        frame_count = 0
        last_stable_target = None
        target_lock_start = None
        
        try:
            while self.is_running:
                ret, frame = self.cap.read()
                if not ret:
                    print("[DartLaserSystem] ❌ Kamera görüntüsü alınamıyor!")
                    break
                
                frame_count += 1
                
                # Dart tespiti yap
                detections = self.dart_detector.detect_darts(frame)
                self.stats['targets_detected'] += len(detections)
                
                # En iyi dart'ı seç
                best_dart = None
                if detections:
                    # Güven değerine göre filtrele
                    valid_darts = [d for d in detections if d['confidence'] >= self.target_confidence_threshold]
                    
                    if valid_darts:
                        # En yakın merkezde olanı seç (daha kararlı hedefleme için)
                        best_dart = min(valid_darts, key=lambda d: 
                            abs(d['center'][0] - self.calibration['center_x']) + 
                            abs(d['center'][1] - self.calibration['center_y'])
                        )
                
                # Görselleştirme
                display_frame = frame.copy()
                self.draw_targeting_info(display_frame, detections, best_dart)
                
                # Hedef takibi
                if best_dart:
                    current_center = best_dart['center']
                    
                    # Kararlılık kontrolü
                    if self.is_target_stable(current_center, 
                                           last_stable_target['center'] if last_stable_target else None):
                        
                        if last_stable_target is None or target_lock_start is None:
                            # Yeni kararlı hedef
                            target_lock_start = time.time()
                            print(f"[DartLaserSystem] 🔒 Hedef kilitlendi, bekleniyor...")
                        
                        # Kilitlenme süresini kontrol et
                        lock_duration = time.time() - target_lock_start
                        
                        if lock_duration >= self.target_lock_time:
                            # Hedef yeterince uzun süre kararlı - ateş et!
                            if self.track_and_fire(best_dart):
                                self.stats['targets_locked'] += 1
                                self.stats['accuracy'] = (self.stats['targets_locked'] / max(1, self.stats['laser_fires'])) * 100
                            
                            # Reset
                            target_lock_start = None
                            last_stable_target = None
                            time.sleep(2)  # Bir sonraki hedef için bekle
                        else:
                            # Hala bekliyoruz
                            remaining = self.target_lock_time - lock_duration
                            cv2.putText(display_frame, f"LOCKING: {remaining:.1f}s", 
                                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    else:
                        # Hedef kararsız - resetle
                        target_lock_start = None
                    
                    last_stable_target = best_dart
                else:
                    # Hedef yok
                    target_lock_start = None
                    last_stable_target = None
                
                # Display
                cv2.imshow('BARLAS Dart Laser Targeting', display_frame)
                
                # Klavye kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
                elif key == ord(' '):  # SPACE - Manuel ateş
                    self.arduino_controller.fire_laser(1.0)
                elif key == ord('c'):  # C - Kalibrasyon
                    self.calibration_mode()
                
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def draw_targeting_info(self, frame, detections: List[Dict], best_dart: Optional[Dict]):
        """Hedefleme bilgilerini frame üzerine çizer"""
        
        # Tüm tespitleri çiz
        for detection in detections:
            x, y, w, h = detection['bbox']
            center = detection['center']
            confidence = detection['confidence']
            
            # Renk seç
            if detection == best_dart:
                color = (0, 255, 0)  # Yeşil - seçili hedef
                thickness = 3
            elif confidence >= self.target_confidence_threshold:
                color = (0, 255, 255)  # Sarı - geçerli hedef
                thickness = 2
            else:
                color = (0, 128, 255)  # Turuncu - düşük güven
                thickness = 1
            
            # Bounding box
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, thickness)
            
            # Merkez noktası
            cv2.circle(frame, center, 8, color, -1)
            
            # Güven değeri
            cv2.putText(frame, f"{confidence:.2f}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Crosshair (merkez)
        center_x, center_y = self.calibration['center_x'], self.calibration['center_y']
        cv2.line(frame, (center_x-20, center_y), (center_x+20, center_y), (255, 255, 255), 2)
        cv2.line(frame, (center_x, center_y-20), (center_x, center_y+20), (255, 255, 255), 2)
        
        # Sistem bilgileri
        arduino_status = self.arduino_controller.get_status()
        pan, tilt = arduino_status.get('pan', 0), arduino_status.get('tilt', 0)
        
        info_lines = [
            f"Pan: {pan:.1f}°  Tilt: {tilt:.1f}°",
            f"Targets: {len(detections)}  Fired: {self.stats['laser_fires']}",
            f"Accuracy: {self.stats['accuracy']:.1f}%"
        ]
        
        for i, line in enumerate(info_lines):
            cv2.putText(frame, line, (10, 30 + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    def calibration_mode(self):
        """Kalibrasyon modunu başlatır"""
        print("[DartLaserSystem] 🎯 Kalibrasyon modu - Ok tuşları ile ayarlayın, ESC ile çıkın")
        
        calibrating = True
        while calibrating:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Kalibrasyon bilgilerini göster
            cv2.putText(frame, "CALIBRATION MODE", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(frame, "Arrow keys: Move servo, ESC: Exit", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Crosshair
            center_x, center_y = self.calibration['center_x'], self.calibration['center_y']
            cv2.line(frame, (center_x-30, center_y), (center_x+30, center_y), (0, 255, 255), 3)
            cv2.line(frame, (center_x, center_y-30), (center_x, center_y+30), (0, 255, 255), 3)
            
            cv2.imshow('BARLAS Dart Laser Targeting', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                calibrating = False
            elif key == 82:  # Up arrow
                self.arduino_controller.move_relative(0, -5)
            elif key == 84:  # Down arrow
                self.arduino_controller.move_relative(0, 5)
            elif key == 81:  # Left arrow
                self.arduino_controller.move_relative(-5, 0)
            elif key == 83:  # Right arrow
                self.arduino_controller.move_relative(5, 0)
            elif key == ord(' '):  # Space - ateş
                self.arduino_controller.fire_laser(0.5)
    
    def cleanup(self):
        """Sistem temizliği"""
        print("[DartLaserSystem] 🧹 Sistem temizleniyor...")
        
        self.is_running = False
        
        if self.cap:
            self.cap.release()
        
        if hasattr(self.arduino_controller, 'disconnect'):
            self.arduino_controller.disconnect()
        
        cv2.destroyAllWindows()
        print("[DartLaserSystem] ✅ Sistem temizlendi")


# Test fonksiyonu
if __name__ == "__main__":
    print("🎯 BARLAS Dart Laser Targeting System Test")
    print("=" * 50)
    
    try:
        # Sistem başlat
        targeting_system = DartLaserTargetingSystem()
        
        # Hedefleme başlat
        targeting_system.start_targeting()
        
    except KeyboardInterrupt:
        print("\\n[Test] Kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"[Test] Hata: {e}")
    
    print("✅ Test tamamlandı!")
