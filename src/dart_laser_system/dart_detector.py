"""
BARLAS Dart Detector
YOLO tabanlı dart tespit sistemi
Bağımsız çalışabilen modül
"""
import cv2
import numpy as np
import time
import math
import os
import sys
from typing import List, Dict, Optional, Tuple

# BARLAS YOLO modülünü import et
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

try:
    from src.dart_recognize.yolo_predictions import YOLOPredictions
    YOLO_AVAILABLE = True
    print("✅ YOLO Dart Recognition modülü yüklendi!")
except ImportError:
    try:
        from dart_recognize.yolo_predictions import YOLOPredictions
        YOLO_AVAILABLE = True
        print("✅ YOLO Dart Recognition modülü yüklendi!")
    except ImportError as e:
        YOLO_AVAILABLE = False
        print(f"⚠️ YOLO modülü bulunamadı: {e}")
        print("Test modu - simülasyon dartları kullanılacak")
        
        # Dummy YOLO class with simulated dart detections
        class YOLOPredictions:
            def __init__(self):
                self.detection_counter = 0
                self.dart_positions = [
                    {'bbox': [300, 200, 40, 60], 'confidence': 0.8},  # Merkez-sol
                    {'bbox': [400, 250, 35, 55], 'confidence': 0.7},  # Merkez-sağ
                    {'bbox': [320, 180, 38, 58], 'confidence': 0.9},  # Üst-merkez
                    {'bbox': [280, 300, 42, 62], 'confidence': 0.6},  # Alt-sol
                ]
            
            def get_detections(self, frame):
                # Her 60 frame'de bir farklı dart pozisyonu döndür
                self.detection_counter += 1
                
                # %70 ihtimalle dart tespit et
                import random
                if random.random() < 0.7:
                    # Dönen dart pozisyonu seç
                    dart_idx = (self.detection_counter // 60) % len(self.dart_positions)
                    selected_dart = self.dart_positions[dart_idx].copy()
                    
                    # Küçük rastgele hareket ekle (gerçekçilik için)
                    noise_x = random.randint(-5, 5)
                    noise_y = random.randint(-5, 5)
                    selected_dart['bbox'][0] += noise_x
                    selected_dart['bbox'][1] += noise_y
                    
                    return [selected_dart]
                else:
                    # Bazen dart bulunamaz
                    return []


class DartDetector:
    """
    YOLO tabanlı dart tespit sistemi
    Bağımsız çalışabilir ve diğer modüllerle entegre olabilir
    """
    
    def __init__(self, confidence_threshold=0.5):
        """
        Dart Detector
        
        Args:
            confidence_threshold: Minimum dart güven eşiği
        """
        self.confidence_threshold = confidence_threshold
        
        # YOLO detector - global YOLO_AVAILABLE kullan
        global YOLO_AVAILABLE
        
        if YOLO_AVAILABLE:
            try:
                # Gerçek YOLO modeli ile başlat
                current_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                model_path = os.path.join(current_dir, "src", "dart_recognize", "Model", "weights", "best.onnx")
                data_path = os.path.join(current_dir, "src", "dart_recognize", "data.yaml")
                
                self.yolo_detector = YOLOPredictions(onnx_model_path=model_path, data_yaml_path=data_path)
                print("[DartDetector] ✅ YOLO dart detector yüklendi")
                print(f"[DartDetector] Model: {model_path}")
                print(f"[DartDetector] Config: {data_path}")
            except Exception as e:
                print(f"[DartDetector] ❌ YOLO yükleme hatası: {e}")
                print("[DartDetector] Simülasyon moduna geçiliyor...")
                YOLO_AVAILABLE = False
                self.yolo_detector = YOLOPredictions()  # Fallback to dummy
        
        if not YOLO_AVAILABLE:
            self.yolo_detector = YOLOPredictions()  # Dummy
            print("[DartDetector] ⚠️ YOLO simülasyon modu")
        
        # İstatistikler
        self.total_detections = 0
        self.valid_detections = 0
        self.last_detection_time = 0
        
        # Hough Circle parametreleri - optimize edilebilir
        self.hough_params = {
            'dp': 1,
            'minDist_ratio': 0.3,      # dart boyutuna orantılı
            'param1': 60,              # Canny edge threshold
            'param2': 25,              # Accumulator threshold  
            'minRadius_ratio': 0.125,  # dart boyutuna orantılı (1/8)
            'blur_kernel': 9,          # Gaussian blur kernel boyutu
            'blur_sigma': 2            # Gaussian blur sigma
        }
        
        # Kamera-servo kalibrasyonu (yolo_arduino_dart_system.py'den alındı)
        self.calibration = {
            'frame_width': 640,
            'frame_height': 480,
            'horizontal_fov': 60,    # Kamera yatay görüş açısı
            'vertical_fov': 45,      # Kamera dikey görüş açısı
            'offset_x': 0,           # Kalibrasyon offset'i
            'offset_y': 0
        }
        
        print(f"[DartDetector] Hazır - Güven eşiği: {confidence_threshold}")
    
    def detect_darts(self, frame) -> List[Dict]:
        """
        Frame'de dart'ları tespit eder
        
        Args:
            frame: OpenCV BGR frame
            
        Returns:
            List[Dict]: Tespit edilen dart'lar
                - bbox: [x, y, w, h]
                - confidence: float
                - center: (x, y) merkez koordinatı
        """
        
        try:
            # Frame'i sakla (Hough Circle için)
            self._last_frame = frame.copy()
            
            # YOLO ile tespit yap
            raw_detections = self.yolo_detector.get_detections(frame)
            self.total_detections += len(raw_detections)
            
            # Güvenli dart'ları filtrele
            valid_darts = []
            
            for detection in raw_detections:
                if detection['confidence'] >= self.confidence_threshold:
                    
                    x, y, w, h = detection['bbox']
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    dart_info = {
                        'bbox': detection['bbox'],
                        'confidence': detection['confidence'],
                        'center': (center_x, center_y),
                        'area': w * h,
                        'timestamp': time.time()
                    }
                    
                    valid_darts.append(dart_info)
                    self.valid_detections += 1
            
            # Son güncelleme zamanı
            self.last_detection_time = time.time()
            
            return valid_darts
            
        except Exception as e:
            print(f"[DartDetector] Tespit hatası: {e}")
            return []
    
    def get_stable_dart(self, detections: List[Dict], previous_center: Optional[Tuple] = None, 
                       stability_threshold: int = 30) -> Optional[Dict]:
        """
        En kararlı dart'ı seçer
        
        Args:
            detections: Tespit edilen dart'lar listesi
            previous_center: Önceki hedef merkezi (x, y)
            stability_threshold: Kararlılık eşiği (piksel)
            
        Returns:
            En kararlı dart veya None
        """
        if not detections:
            return None
            
        # Tek dart varsa onu döndür
        if len(detections) == 1:
            return detections[0]
        
        # En yüksek güvenli dart'ı bul
        best_dart = max(detections, key=lambda d: d['confidence'])
        
        # Eğer önceki merkez varsa, ona en yakın olan kararlı dart'ı seç
        if previous_center is not None:
            prev_x, prev_y = previous_center
            
            stable_candidates = []
            for dart in detections:
                center_x, center_y = dart['center']
                distance = math.sqrt((center_x - prev_x)**2 + (center_y - prev_y)**2)
                
                if distance < stability_threshold:
                    dart['stability_distance'] = distance
                    stable_candidates.append(dart)
            
            # En kararlı ve en güvenli olanı seç
            if stable_candidates:
                best_dart = max(stable_candidates, 
                              key=lambda d: (1.0 - d['stability_distance'] / stability_threshold) * d['confidence'])
        
        return best_dart

    def get_best_dart(self, detections: List[Dict], frame=None) -> Optional[Dict]:
        """
        En iyi tek dart'ı seçer (en yüksek güven + Hough Circle ile merkez düzeltmesi)
        
        Args:
            detections: Tespit edilen dart'lar
            frame: Hough Circle için frame (isteğe bağlı)
            
        Returns:
            En iyi dart veya None
        """
        if not detections:
            return None
        
        # Frame'i sakla
        if frame is not None:
            self._last_frame = frame
        
        # Tek dart varsa onu döndür
        if len(detections) == 1:
            return self.refine_dart_center(detections[0])
        
        # En yüksek güvenli dart'ı seç
        best_dart = max(detections, key=lambda d: d['confidence'])
        
        # Hough Circle ile merkezi düzelt
        return self.refine_dart_center(best_dart)
    
    def refine_dart_center(self, dart: Dict) -> Dict:
        """
        Hough Circle algoritması ile dart merkezini hassaslaştırır
        
        Args:
            dart: YOLO'dan gelen dart bilgisi
            
        Returns:
            Merkezi düzeltilmiş dart bilgisi
        """
        # Eğer son frame yoksa orijinal dart'ı döndür
        if not hasattr(self, '_last_frame') or self._last_frame is None:
            return dart
            
        try:
            frame = self._last_frame
            x, y, w, h = dart['bbox']
            
            # Dart bölgesini kırp
            dart_region = frame[y:y+h, x:x+w]
            
            if dart_region.size == 0:
                return dart
            
            # Gri tonlamaya çevir
            gray = cv2.cvtColor(dart_region, cv2.COLOR_BGR2GRAY)
            
            # Gaussian blur uygula - parametrik
            kernel_size = self.hough_params['blur_kernel']
            sigma = self.hough_params['blur_sigma']
            blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), sigma)
            
            # Dinamik parametreler hesapla
            min_dist = int(min(w, h) * self.hough_params['minDist_ratio'])
            min_radius = max(3, int(min(w, h) * self.hough_params['minRadius_ratio']))
            max_radius = min(w, h) // 2
            
            # Hough Circle parametreleri - optimize edilmiş
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=self.hough_params['dp'],
                minDist=min_dist,
                param1=self.hough_params['param1'],
                param2=self.hough_params['param2'],
                minRadius=min_radius,
                maxRadius=max_radius
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
                    
                    # Dart bilgisini güncelle
                    refined_dart = dart.copy()
                    refined_dart['center'] = (global_cx, global_cy)
                    refined_dart['hough_radius'] = r
                    refined_dart['refined'] = True
                    
                    print(f"[DartDetector] 🎯 Hough Circle: Merkez ({global_cx}, {global_cy}), R={r}")
                    return refined_dart
        
        except Exception as e:
            print(f"[DartDetector] Hough Circle hatası: {e}")
        
        # Hata durumunda orijinal dart'ı döndür
        return dart
    
    def set_confidence_threshold(self, new_threshold: float):
        """Güven eşiğini günceller"""
        self.confidence_threshold = max(0.1, min(1.0, new_threshold))
        print(f"[DartDetector] Güven eşiği güncellendi: {self.confidence_threshold}")
    
    def set_hough_params(self, **kwargs):
        """
        Hough Circle parametrelerini günceller
        
        Kullanılabilir parametreler:
        - param1: Canny edge threshold (varsayılan: 60)
        - param2: Accumulator threshold (varsayılan: 25) 
        - minDist_ratio: Min distance ratio to dart size (varsayılan: 0.3)
        - minRadius_ratio: Min radius ratio to dart size (varsayılan: 0.125)
        - blur_kernel: Gaussian blur kernel size (varsayılan: 9)
        - blur_sigma: Gaussian blur sigma (varsayılan: 2)
        """
        for key, value in kwargs.items():
            if key in self.hough_params:
                self.hough_params[key] = value
                print(f"[DartDetector] Hough parametresi güncellendi: {key} = {value}")
            else:
                print(f"[DartDetector] ⚠️ Bilinmeyen Hough parametresi: {key}")
    
    def get_hough_params(self) -> Dict:
        """Mevcut Hough parametrelerini döndürür"""
        return self.hough_params.copy()
    
    def pixel_to_servo_angle(self, pixel_x: int, pixel_y: int, current_pan: float = 90, current_tilt: float = 90) -> Tuple[float, float]:
        """
        Piksel koordinatlarını servo açılarına çevir (yolo_arduino_dart_system.py'den optimize edildi)
        
        Args:
            pixel_x, pixel_y: Hedef piksel koordinatları
            current_pan, current_tilt: Mevcut servo pozisyonları
            
        Returns:
            (target_pan, target_tilt) servo açıları
        """
        # Kamera merkezi
        center_x = self.calibration['frame_width'] / 2
        center_y = self.calibration['frame_height'] / 2
        
        # Offset uygula
        offset_x = pixel_x - center_x + self.calibration['offset_x']
        offset_y = pixel_y - center_y + self.calibration['offset_y']
        
        # Açı hesapla
        pan_adjustment = (offset_x / center_x) * (self.calibration['horizontal_fov'] / 2)
        tilt_adjustment = -(offset_y / center_y) * (self.calibration['vertical_fov'] / 2)
        
        target_pan = current_pan + pan_adjustment
        target_tilt = current_tilt + tilt_adjustment
        
        # Servo limitleri uygula
        target_pan = max(0, min(180, target_pan))
        target_tilt = max(20, min(160, target_tilt))
        
        return target_pan, target_tilt
    
    def set_camera_calibration(self, frame_width: int = 640, frame_height: int = 480, 
                              horizontal_fov: float = 60, vertical_fov: float = 45,
                              offset_x: float = 0, offset_y: float = 0):
        """Kamera kalibrasyon parametrelerini ayarla"""
        self.calibration.update({
            'frame_width': frame_width,
            'frame_height': frame_height, 
            'horizontal_fov': horizontal_fov,
            'vertical_fov': vertical_fov,
            'offset_x': offset_x,
            'offset_y': offset_y
        })
        print(f"[DartDetector] Kalibrasyon güncellendi: {frame_width}x{frame_height}, FOV: {horizontal_fov}°x{vertical_fov}°")
    
    def get_detection_stats(self) -> Dict:
        """Tespit istatistiklerini döndürür"""
        accuracy = (self.valid_detections / max(1, self.total_detections)) * 100
        
        return {
            'total_detections': self.total_detections,
            'valid_detections': self.valid_detections,
            'accuracy': f"{accuracy:.1f}%",
            'confidence_threshold': self.confidence_threshold,
            'last_detection_time': self.last_detection_time
        }
    
    def draw_detections(self, frame, detections: List[Dict], stable_dart: Optional[Dict] = None):
        """
        Frame üzerine tespit sonuçlarını çizer
        
        Args:
            frame: OpenCV frame
            detections: Tespit sonuçları
            stable_dart: Kararlı hedef dart (varsa)
            
        Returns:
            Çizimli frame
        """
        display_frame = frame.copy()
        
        # Tüm tespitleri çiz
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            center = detection['center']
            
            # Bbox çiz
            color = (0, 255, 0) if confidence >= self.confidence_threshold else (0, 255, 255)
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, 2)
            
            # Merkez noktası
            cv2.circle(display_frame, center, 5, color, -1)
            
            # Güven değeri
            cv2.putText(display_frame, f"{confidence:.2f}", 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Kararlı hedefi vurgula
        if stable_dart:
            cx, cy = stable_dart['center']
            cv2.circle(display_frame, (cx, cy), 20, (255, 255, 0), 3)
            cv2.putText(display_frame, "STABLE", (cx-30, cy-25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Info
        info_text = f"Detections: {len(detections)} | Threshold: {self.confidence_threshold:.1f}"
        cv2.putText(display_frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return display_frame


def test_dart_detector():
    """Dart detector test fonksiyonu"""
    print("🎯 BARLAS DART DETECTOR TEST")
    print("=" * 50)
    
    try:
        # Detector oluştur
        detector = DartDetector(confidence_threshold=0.6)
        
        # Kamera aç
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Kamera açılamadı!")
            return
        
        print("✅ Test başlatıldı")
        print("🎮 Kontroller:")
        print("  'q': Çıkış")
        print("  '+': Güven eşiği arttır")
        print("  '-': Güven eşiği azalt")
        print("  's': İstatistikleri göster")
        
        previous_center = None
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Frame okunamadı")
                continue
            
            # Dart tespiti
            detections = detector.detect_darts(frame)
            
            # Kararlı dart seç
            stable_dart = None
            if detections:
                stable_dart = detector.get_stable_dart(detections, previous_center, 30)
                
            if stable_dart:
                previous_center = stable_dart['center']
            
            # Çizimleri yap
            display_frame = detector.draw_detections(frame, detections, stable_dart)
            
            # Hedef bilgisi
            if stable_dart:
                cx, cy = stable_dart['center']
                cv2.circle(display_frame, (cx, cy), 20, (255, 255, 0), 3)
                cv2.putText(display_frame, "STABLE TARGET", (cx-60, cy-30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Çerçeve merkezi
            h, w = display_frame.shape[:2]
            cv2.circle(display_frame, (w//2, h//2), 3, (255, 0, 0), -1)
            cv2.line(display_frame, (w//2-20, h//2), (w//2+20, h//2), (255, 0, 0), 1)
            cv2.line(display_frame, (w//2, h//2-20), (w//2, h//2+20), (255, 0, 0), 1)
            
            cv2.imshow('BARLAS Dart Detector Test', display_frame)
            
            # Klavye kontrolü
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('+'):
                new_threshold = min(1.0, detector.confidence_threshold + 0.1)
                detector.set_confidence_threshold(new_threshold)
            elif key == ord('-'):
                new_threshold = max(0.1, detector.confidence_threshold - 0.1)
                detector.set_confidence_threshold(new_threshold)
            elif key == ord('s'):
                stats = detector.get_detection_stats()
                print(f"\n📊 İstatistikler:")
                for key, value in stats.items():
                    print(f"  {key}: {value}")
        
        print(f"\n✅ Test tamamlandı!")
        
        # Final istatistikleri
        stats = detector.get_detection_stats()
        print(f"\n📊 Final İstatistikleri:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    test_dart_detector()
