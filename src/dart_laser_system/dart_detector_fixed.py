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
    
    # Dummy YOLO class
    class YOLOPredictions:
        def get_detections(self, frame):
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
        
        # YOLO detector - global değişkeni güvenli kullan
        global YOLO_AVAILABLE
        
        if YOLO_AVAILABLE:
            try:
                # Gerçek YOLO modeli ile başlat
                current_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                model_path = os.path.join(current_dir, "src", "dart_recognize", "Model", "weights", "best.onnx")
                data_path = os.path.join(current_dir, "src", "dart_recognize", "data.yaml")
                
                self.yolo_detector = YOLOPredictions(onnx_model_path=model_path, data_yaml_path=data_path)
                self.yolo_available = True
                print("[DartDetector] ✅ YOLO dart detector yüklendi")
                print(f"[DartDetector] Model: {model_path}")
                print(f"[DartDetector] Config: {data_path}")
            except Exception as e:
                print(f"[DartDetector] ❌ YOLO yükleme hatası: {e}")
                print("[DartDetector] Simülasyon moduna geçiliyor...")
                self.yolo_available = False
                self.yolo_detector = YOLOPredictions()  # Fallback to dummy
        else:
            self.yolo_available = False
            self.yolo_detector = YOLOPredictions()  # Dummy
            print("[DartDetector] ⚠️ YOLO simülasyon modu")
        
        # İstatistikler
        self.total_detections = 0
        self.valid_detections = 0
        self.last_detection_time = 0
        
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

    def get_best_dart(self, detections: List[Dict]) -> Optional[Dict]:
        """
        En iyi dart'ı seçer (en yüksek güven)
        
        Args:
            detections: Tespit edilen dart'lar
            
        Returns:
            En iyi dart veya None
        """
        if not detections:
            return None
        
        return max(detections, key=lambda d: d['confidence'])
    
    def set_confidence_threshold(self, new_threshold: float):
        """Güven eşiğini günceller"""
        self.confidence_threshold = max(0.1, min(1.0, new_threshold))
        print(f"[DartDetector] Güven eşiği güncellendi: {self.confidence_threshold}")
    
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
