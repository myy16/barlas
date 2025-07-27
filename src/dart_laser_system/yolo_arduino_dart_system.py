"""
BARLAS Arduino Dart Laser System
YOLO Dart Detection + Hough Circle + Arduino Pan-Tilt
Gerçek dart tanıma sistemi ile Arduino kontrolü
"""
import cv2
import numpy as np
import time
import math
import os
import sys
from typing import Optional, Tuple, List

# BARLAS YOLO modülünü import et
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from dart_recognize.yolo_predictions import YOLOPredictions
    YOLO_AVAILABLE = True
    print("✅ YOLO Dart Recognition modülü yüklendi!")
except ImportError as e:
    YOLO_AVAILABLE = False
    print(f"⚠️ YOLO modülü bulunamadı: {e}")

# Arduino Controller import et
try:
    from arduino_controller import ArduinoPanTiltController
    ARDUINO_AVAILABLE = True
    print("✅ Arduino Controller modülü yüklendi!")
except ImportError as e:
    ARDUINO_AVAILABLE = False
    print(f"⚠️ Arduino Controller modülü bulunamadı: {e}")


class HoughCircleDetector:
    """
    Hough Circle algoritması ile dart merkez tespiti
    YOLO'dan sonra daha hassas merkez bulma
    """
    
    def __init__(self):
        print("[HoughCircle] 🎯 Hough Circle detector başlatıldı")
    
    def find_dart_center(self, frame, dart_bbox):
        """
        Dart bounding box içinde Hough Circle ile merkez bul
        
        Args:
            frame: OpenCV frame
            dart_bbox: [x, y, w, h] YOLO'dan gelen bbox
            
        Returns:
            (center_x, center_y) veya None
        """
        x, y, w, h = dart_bbox
        
        # Dart bölgesini kırp
        dart_region = frame[y:y+h, x:x+w]
        
        if dart_region.size == 0:
            return None
        
        try:
            # Gri tonlamaya çevir
            gray = cv2.cvtColor(dart_region, cv2.COLOR_BGR2GRAY)
            
            # Gaussian blur uygula
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            
            # Hough Circle parametreleri
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,              # Accumulator resolution
                minDist=30,        # Minimum mesafe circle'lar arası
                param1=50,         # Canny edge threshold
                param2=30,         # Accumulator threshold
                minRadius=5,       # Minimum circle radius
                maxRadius=min(w, h)//2  # Maximum circle radius
            )
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                
                # En büyük circle'ı seç (dart merkezi olma ihtimali yüksek)
                best_circle = None
                max_radius = 0
                
                for (cx, cy, r) in circles:
                    if r > max_radius:
                        max_radius = r
                        best_circle = (cx, cy, r)
                
                if best_circle:
                    cx, cy, r = best_circle
                    
                    # Dart region'daki koordinatları global koordinatlara çevir
                    global_cx = x + cx
                    global_cy = y + cy
                    
                    print(f"[HoughCircle] 🎯 Dart merkezi bulundu: ({global_cx}, {global_cy}), r={r}")
                    return (global_cx, global_cy, r)
        
        except Exception as e:
            print(f"[HoughCircle] Hough Circle hatası: {e}")
        
        return None
    
    def draw_circle_detection(self, frame, dart_bbox, circle_result):
        """Circle detection sonucunu çiz"""
        if circle_result:
            cx, cy, r = circle_result
            
            # Circle'ı çiz
            cv2.circle(frame, (cx, cy), r, (255, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 3, (255, 0, 255), -1)
            
            # Merkez çizgisi
            cv2.line(frame, (cx-10, cy), (cx+10, cy), (255, 0, 255), 2)
            cv2.line(frame, (cx, cy-10), (cx, cy+10), (255, 0, 255), 2)
            
            # Metin
            cv2.putText(frame, f"Center: ({cx},{cy})", (cx-50, cy-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)


class ArduinoSimulator:
    """
    Arduino Pan-Tilt simülatörü
    """
    
    def __init__(self):
        self.pan_position = 90
        self.tilt_position = 90
        self.laser_active = False
        self.is_connected = True
        
        # Kalibrasyon
        self.calibration_offset_x = 0
        self.calibration_offset_y = 0
        
        print("[ArduinoSim] 🎮 Arduino Simülatörü başlatıldı")
    
    def move_to_position(self, pan_angle: float, tilt_angle: float) -> bool:
        """Simüle edilmiş servo hareketi"""
        # Sınırları kontrol et
        pan_angle = max(10, min(170, pan_angle))
        tilt_angle = max(30, min(150, tilt_angle))
        
        self.pan_position = pan_angle
        self.tilt_position = tilt_angle
        
        print(f"[ArduinoSim] 🔄 SERVO KOMUTU: MOVE,{int(pan_angle)},{int(tilt_angle)}")
        return True
    
    def enable_laser(self) -> bool:
        """Lazer açma komutu"""
        self.laser_active = True
        print("[ArduinoSim] 🔴 ARDUINO KOMUTU: LASER,ON")
        return True
    
    def disable_laser(self) -> bool:
        """Lazer kapama komutu"""
        self.laser_active = False
        print("[ArduinoSim] ⚫ ARDUINO KOMUTU: LASER,OFF")
        return True
    
    def center_position(self) -> bool:
        """Merkez pozisyon komutu"""
        print("[ArduinoSim] 🏠 ARDUINO KOMUTU: CENTER")
        return self.move_to_position(90, 90)
    
    def pixel_to_angle(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> Tuple[float, float]:
        """Piksel koordinatlarını servo açılarına çevir"""
        # Kamera merkezi
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # Offset uygula
        offset_x = pixel_x - center_x + self.calibration_offset_x
        offset_y = pixel_y - center_y + self.calibration_offset_y
        
        # Kamera FOV (60° yatay, 45° dikey)
        horizontal_fov = 60
        vertical_fov = 45
        
        # Açı hesapla
        pan_adjustment = (offset_x / center_x) * (horizontal_fov / 2)
        tilt_adjustment = -(offset_y / center_y) * (vertical_fov / 2)
        
        target_pan = self.pan_position + pan_adjustment
        target_tilt = self.tilt_position + tilt_adjustment
        
        return target_pan, target_tilt
    
    def aim_at_pixel(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> bool:
        """Piksele nişan al"""
        target_pan, target_tilt = self.pixel_to_angle(pixel_x, pixel_y, frame_width, frame_height)
        
        print(f"[ArduinoSim] 🎯 HEDEF: Piksel ({pixel_x}, {pixel_y}) -> Servo ({target_pan:.1f}°, {target_tilt:.1f}°)")
        
        if self.move_to_position(target_pan, target_tilt):
            return self.enable_laser()
        return False


class YOLOArduinoDartSystem:
    """
    YOLO + Hough Circle + Arduino Dart Laser System
    """
    
    def __init__(self, camera_index=0):
        print("[YOLOArduinoSystem] 🎯 YOLO+Arduino Dart Sistem başlatılıyor...")
        
        # YOLO Dart Detector
        if YOLO_AVAILABLE:
            try:
                # Doğru model path'i ile YOLO'yu başlat
                current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                model_path = os.path.join(current_dir, "dart_recognize", "Model", "weights", "best.onnx")
                data_path = os.path.join(current_dir, "dart_recognize", "data.yaml")
                
                self.yolo_detector = YOLOPredictions(onnx_model_path=model_path, data_yaml_path=data_path)
                print("[YOLOArduinoSystem] ✅ YOLO Dart Detector yüklendi")
            except Exception as e:
                print(f"[YOLOArduinoSystem] ❌ YOLO yükleme hatası: {e}")
                raise
        else:
            print("[YOLOArduinoSystem] ❌ YOLO bulunamadı!")
            return
        
        # Hough Circle Detector
        self.hough_detector = HoughCircleDetector()
        
        # Arduino Controller - Gerçek veya Simülatör
        self.arduino_controller = self.initialize_arduino()
        
        # Kamera
        self.camera_index = camera_index
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480
        
        # Hedefleme durumu
        self.current_dart_target = None
        self.targeting_start_time = 0
        self.lock_duration = 2.0  # 2 saniye kilitlenme
        self.confidence_threshold = 0.5
        
        print("[YOLOArduinoSystem] 🎮 Sistem hazır!")
    
    def initialize_arduino(self):
        """Arduino Controller başlat - Gerçek veya Simülatör"""
        if ARDUINO_AVAILABLE:
            # Önce gerçek Arduino'yu dene
            for port in ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']:
                try:
                    print(f"[YOLOArduinoSystem] 🔍 Arduino bağlantısı deneniyor: {port}")
                    arduino = ArduinoPanTiltController(port=port)
                    if arduino.connect():
                        print(f"[YOLOArduinoSystem] ✅ Gerçek Arduino bağlandı: {port}")
                        return arduino
                    else:
                        arduino.disconnect()
                except Exception as e:
                    continue
            
            print("[YOLOArduinoSystem] ⚠️ Gerçek Arduino bulunamadı, simülatör kullanılıyor")
        
        # Simülatör kullan
        return ArduinoSimulator()
    
    def initialize_camera(self):
        """Kamerayı başlat"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"[YOLOArduinoSystem] ❌ Kamera {self.camera_index} açılamadı!")
                return False
            
            # Kamera ayarları
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            # Gerçek çözünürlük
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[YOLOArduinoSystem] ✅ Kamera: {self.frame_width}x{self.frame_height}")
            return True
            
        except Exception as e:
            print(f"[YOLOArduinoSystem] ❌ Kamera hatası: {e}")
            return False
    
    def run_dart_targeting(self):
        """Ana dart hedefleme döngüsü"""
        
        if not self.initialize_camera():
            return
        
        print("\n🚀 YOLO+ARDUINO DART TARGETING BAŞLIYOR!")
        print("=" * 60)
        print("📋 Kontroller:")
        print("  'q' - Çıkış")
        print("  'space' - Manuel lazer açma/kapama") 
        print("  'c' - Merkez pozisyon")
        print("  'wasd' - Manuel servo hareket")
        print("  '+/-' - Güven eşiği ayarı")
        print("=" * 60)
        print("🎯 Kameraya DART gösterin!")
        print()
        
        # Mouse callback
        cv2.namedWindow('BARLAS YOLO+Arduino Dart Targeting')
        cv2.setMouseCallback('BARLAS YOLO+Arduino Dart Targeting', self.mouse_callback)
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            try:
                current_time = time.time()
                
                # YOLO ile dart tespiti
                dart_detections = self.yolo_detector.get_detections(frame)
                
                # Güvenli dart'ları filtrele
                valid_darts = [d for d in dart_detections 
                              if d['confidence'] >= self.confidence_threshold]
                
                best_dart_center = None
                best_dart_info = None
                
                if valid_darts:
                    # En iyi dart'ı seç
                    best_dart = max(valid_darts, 
                                   key=lambda d: d['confidence'] * (d['bbox'][2] * d['bbox'][3]))
                    
                    # Hough Circle ile hassas merkez tespiti
                    circle_result = self.hough_detector.find_dart_center(frame, best_dart['bbox'])
                    
                    if circle_result:
                        # Hough Circle sonucu kullan
                        center_x, center_y, radius = circle_result
                        best_dart_center = (center_x, center_y)
                        best_dart_info = {
                            'center': best_dart_center,
                            'confidence': best_dart['confidence'],
                            'method': 'Hough Circle',
                            'radius': radius,
                            'bbox': best_dart['bbox']
                        }
                    else:
                        # YOLO bbox merkezi kullan
                        x, y, w, h = best_dart['bbox']
                        center_x = x + w // 2
                        center_y = y + h // 2
                        best_dart_center = (center_x, center_y)
                        best_dart_info = {
                            'center': best_dart_center,
                            'confidence': best_dart['confidence'],
                            'method': 'YOLO BBox',
                            'bbox': best_dart['bbox']
                        }
                
                # Hedefleme logic
                if best_dart_center:
                    target_x, target_y = best_dart_center
                    
                    # Hedef değişti mi?
                    if self.current_dart_target is None:
                        self.current_dart_target = best_dart_center
                        self.targeting_start_time = current_time
                        print(f"[YOLOArduinoSystem] 🎯 YENİ DART TESPIT: {best_dart_info['method']} - ({target_x}, {target_y}), Güven: {best_dart_info['confidence']:.2f}")
                    
                    else:
                        # Hedef kararlılık kontrolü
                        prev_x, prev_y = self.current_dart_target
                        distance = math.sqrt((target_x - prev_x)**2 + (target_y - prev_y)**2)
                        
                        if distance < 25:  # Kararlı hedef (25 piksel tolerans)
                            lock_time = current_time - self.targeting_start_time
                            
                            # Kilitlenme süresi doldu mu?
                            if lock_time >= self.lock_duration:
                                print(f"[YOLOArduinoSystem] 🔥 DART KİLİTLENDİ! {best_dart_info['method']} Merkez: ({target_x}, {target_y})")
                                print(f"[YOLOArduinoSystem] 🚀 Arduino'ya hedefleme komutu gönderiliyor...")
                                
                                # Arduino'ya hedefleme komutu gönder
                                self.arduino_controller.aim_at_pixel(target_x, target_y, self.frame_width, self.frame_height)
                                
                                # Yeni hedef ara
                                self.current_dart_target = None
                                self.targeting_start_time = 0
                        
                        else:
                            # Hedef değişti
                            self.current_dart_target = best_dart_center
                            self.targeting_start_time = current_time
                            print(f"[YOLOArduinoSystem] 🔄 Dart pozisyon değişti: ({target_x}, {target_y})")
                
                else:
                    # Dart bulunamadı
                    if self.current_dart_target:
                        print("[YOLOArduinoSystem] ❌ Dart kaybedildi")
                        self.current_dart_target = None
                        self.arduino_controller.disable_laser()
                
                # Görsel çizim
                display_frame = self.draw_targeting_info(frame, dart_detections, valid_darts, best_dart_info, current_time)
                cv2.imshow('BARLAS YOLO+Arduino Dart Targeting', display_frame)
                
                # Klavye kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    if self.arduino_controller.laser_active:
                        self.arduino_controller.disable_laser()
                    else:
                        self.arduino_controller.enable_laser()
                elif key == ord('c'):
                    self.arduino_controller.center_position()
                elif key == ord('w'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position,
                        self.arduino_controller.tilt_position - 5
                    )
                elif key == ord('s'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position,
                        self.arduino_controller.tilt_position + 5
                    )
                elif key == ord('a'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position - 5,
                        self.arduino_controller.tilt_position
                    )
                elif key == ord('d'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position + 5,
                        self.arduino_controller.tilt_position
                    )
                elif key == ord('+') or key == ord('='):
                    self.confidence_threshold = min(1.0, self.confidence_threshold + 0.1)
                    print(f"[YOLOArduinoSystem] Güven eşiği: {self.confidence_threshold:.1f}")
                elif key == ord('-'):
                    self.confidence_threshold = max(0.1, self.confidence_threshold - 0.1)
                    print(f"[YOLOArduinoSystem] Güven eşiği: {self.confidence_threshold:.1f}")
                
            except Exception as e:
                print(f"[YOLOArduinoSystem] Döngü hatası: {e}")
                time.sleep(0.1)
        
        # Temizlik
        self.cap.release()
        cv2.destroyAllWindows()
        print("\n🏁 YOLO+Arduino Dart Targeting sonlandırıldı")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Mouse ile manuel hedefleme"""
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"[YOLOArduinoSystem] 🖱️ Manuel hedef: ({x}, {y})")
            self.arduino_controller.aim_at_pixel(x, y, self.frame_width, self.frame_height)
    
    def draw_targeting_info(self, frame, all_detections, valid_darts, best_dart_info, current_time):
        """Targeting bilgilerini çiz"""
        display_frame = frame.copy()
        
        # Tüm YOLO tespitlerini çiz
        for detection in all_detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            
            # Renk seçimi
            if confidence >= self.confidence_threshold:
                color = (0, 255, 0)  # Yeşil - geçerli
                thickness = 2
            else:
                color = (0, 0, 255)  # Kırmızı - düşük güven
                thickness = 1
            
            # Bounding box
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, thickness)
            
            # Güven skoru
            cv2.putText(display_frame, f"DART: {confidence:.2f}", (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # En iyi dart'ı vurgula
        if best_dart_info:
            center_x, center_y = best_dart_info['center']
            method = best_dart_info['method']
            
            # Merkez noktası
            if method == 'Hough Circle':
                # Hough Circle sonucunu çiz
                if 'radius' in best_dart_info:
                    radius = best_dart_info['radius']
                    cv2.circle(display_frame, (center_x, center_y), radius, (255, 0, 255), 2)
                cv2.circle(display_frame, (center_x, center_y), 5, (255, 0, 255), -1)
                color = (255, 0, 255)  # Magenta
            else:
                cv2.circle(display_frame, (center_x, center_y), 8, (255, 255, 0), -1)
                color = (255, 255, 0)  # Sarı
            
            # Merkez çizgisi
            cv2.line(display_frame, (center_x-15, center_y), (center_x+15, center_y), color, 2)
            cv2.line(display_frame, (center_x, center_y-15), (center_x, center_y+15), color, 2)
            
            # Method bilgisi
            cv2.putText(display_frame, method, (center_x-40, center_y-25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Hedef kilitlenme progress
        if self.current_dart_target and self.targeting_start_time > 0:
            target_x, target_y = self.current_dart_target
            elapsed = current_time - self.targeting_start_time
            progress = min(elapsed / self.lock_duration, 1.0)
            
            # Progress bar
            bar_width = int(80 * progress)
            cv2.rectangle(display_frame, (target_x - 40, target_y + 30),
                         (target_x - 40 + bar_width, target_y + 35), (0, 255, 255), -1)
            cv2.rectangle(display_frame, (target_x - 40, target_y + 30),
                         (target_x + 40, target_y + 35), (255, 255, 255), 2)
            
            # Progress metni
            status = "LOCKED!" if progress >= 1.0 else f"LOCKING {progress*100:.0f}%"
            cv2.putText(display_frame, status, (target_x - 50, target_y + 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Arduino durumu
        arduino_info = [
            f"Arduino: Pan={self.arduino_controller.pan_position:.0f}° Tilt={self.arduino_controller.tilt_position:.0f}°",
            f"Laser: {'ON' if self.arduino_controller.laser_active else 'OFF'}",
            f"YOLO Threshold: {self.confidence_threshold:.1f}",
            f"Valid Darts: {len(valid_darts)}/{len(all_detections)}"
        ]
        
        for i, info in enumerate(arduino_info):
            cv2.putText(display_frame, info, (10, 30 + i*25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Sistem durumu
        status_color = (0, 255, 0) if self.arduino_controller.laser_active else (255, 255, 255)
        status_text = "LAZER AKTIF" if self.arduino_controller.laser_active else "DART ARANIYOR"
        cv2.putText(display_frame, status_text, (10, display_frame.shape[0] - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Frame merkezi
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        cv2.circle(display_frame, (center_x, center_y), 3, (255, 0, 0), -1)
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 1)
        
        # Kontrol bilgileri
        controls = ["Q:Quit", "Space:Laser", "C:Center", "WASD:Move", "+/-:Threshold"]
        for i, control in enumerate(controls):
            cv2.putText(display_frame, control, (display_frame.shape[1] - 200, 30 + i*20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        return display_frame


def main():
    """Ana program"""
    import argparse
    
    # Komut satırı argümanları
    parser = argparse.ArgumentParser(description='BARLAS YOLO+Arduino Dart Targeting System')
    parser.add_argument('--camera', type=int, default=0, help='Kamera index (0=dahili, 1=USB, ...)')
    parser.add_argument('--list-cameras', action='store_true', help='Mevcut kameraları listele')
    args = parser.parse_args()
    
    # Kamera listesi isteniyorsa
    if args.list_cameras:
        print("🎥 Mevcut Kameralar:")
        for i in range(5):
            try:
                import cv2
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        h, w = frame.shape[:2]
                        print(f"  {i}: {'Dahili' if i==0 else 'USB'} Kamera - {w}x{h}")
                cap.release()
            except:
                pass
        return
    
    print("🎯 BARLAS YOLO+ARDUINO DART LASER TARGETING SYSTEM")
    print("=" * 70)
    print("YOLO Dart Detection + Hough Circle + Arduino Pan-Tilt")
    print("=" * 70)
    print(f"📹 Kamera: {args.camera} ({'Dahili' if args.camera==0 else 'USB'})")
    print(f"🔌 Arduino: Otomatik tespit")
    print("=" * 70)
    
    if not YOLO_AVAILABLE:
        print("❌ YOLO modülü bulunamadı! Lütfen dart_recognize modülünü kontrol edin.")
        return
    
    try:
        # Sistem oluştur
        dart_system = YOLOArduinoDartSystem(camera_index=args.camera)
        
        # Sistemi başlat
        dart_system.run_dart_targeting()
        
    except Exception as e:
        print(f"❌ Sistem hatası: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
