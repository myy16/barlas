"""
BARLAS Dart Detector
YOLO tabanlı dart tespit sistemi
Bağımsız çalışabilen modül
"""
import cv2
import numpy as np
import time
import os
import sys
from typing import List, Dict, Optional, Tuple

# BARLAS YOLO modülünü import et
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

try:
    from dart_recognize.yolo_predictions import YOLOPredictions
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("YOLO modülü bulunamadı - test modu")
    
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
        
        # YOLO detector
        if YOLO_AVAILABLE:
            try:
                self.yolo_detector = YOLOPredictions()
                print("[DartDetector] ✅ YOLO dart detector yüklendi")
            except Exception as e:
                print(f"[DartDetector] ❌ YOLO yükleme hatası: {e}")
                raise
        else:
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
            
            if valid_darts:
                self.last_detection_time = time.time()
            
            return valid_darts
            
        except Exception as e:
            print(f"[DartDetector] Tespit hatası: {e}")
            return []
    
    def get_best_dart(self, detections: List[Dict]) -> Optional[Dict]:
        """
        En iyi dart'ı seçer (güven x alan)
        
        Args:
            detections: Dart tespit listesi
            
        Returns:
            Dict veya None: En iyi dart
        """
        
        if not detections:
            return None
        
        # Güven skoru ve alan çarpımına göre sırala
        best_dart = max(detections, 
                       key=lambda d: d['confidence'] * d['area'])
        
        return best_dart
    
    def get_stable_dart(self, detections: List[Dict], 
                       previous_center: Optional[Tuple[int, int]] = None,
                       stability_threshold: int = 30) -> Optional[Dict]:
        """
        Kararlı dart'ı bulur (önceki pozisyona yakın)
        
        Args:
            detections: Dart tespit listesi
            previous_center: Önceki dart merkezi (x, y)
            stability_threshold: Kararlılık mesafe eşiği (piksel)
            
        Returns:
            Dict veya None: Kararlı dart
        """
        
        if not detections:
            return None
        
        if previous_center is None:
            return self.get_best_dart(detections)
        
        prev_x, prev_y = previous_center
        
        # Önceki pozisyona en yakın dart'ı bul
        stable_darts = []
        
        for dart in detections:
            center_x, center_y = dart['center']
            distance = np.sqrt((center_x - prev_x)**2 + (center_y - prev_y)**2)
            
            if distance <= stability_threshold:
                dart['stability_distance'] = distance
                stable_darts.append(dart)
        
        if not stable_darts:
            return None
        
        # En yakın ve en güvenli olanı seç
        best_stable = min(stable_darts, 
                         key=lambda d: d['stability_distance'] - d['confidence'] * 50)
        
        return best_stable
    
    def draw_detections(self, frame, detections: List[Dict], 
                       highlight_dart: Optional[Dict] = None) -> np.ndarray:
        """
        Tespit edilen dart'ları frame üzerine çizer
        
        Args:
            frame: OpenCV BGR frame
            detections: Dart tespit listesi
            highlight_dart: Vurgulanacak özel dart
            
        Returns:
            np.ndarray: Çizimli frame
        """
        
        display_frame = frame.copy()
        
        # Tüm tespitleri çiz
        for dart in detections:
            x, y, w, h = dart['bbox']
            confidence = dart['confidence']
            center_x, center_y = dart['center']
            
            # Renk seçimi
            if highlight_dart and dart == highlight_dart:
                color = (0, 255, 255)  # Sarı - vurgulu
                thickness = 3
            elif confidence >= 0.8:
                color = (0, 255, 0)    # Yeşil - yüksek güven
                thickness = 2
            elif confidence >= 0.6:
                color = (0, 165, 255)  # Turuncu - orta güven
                thickness = 2
            else:
                color = (0, 0, 255)    # Kırmızı - düşük güven
                thickness = 1
            
            # Bounding box
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, thickness)
            
            # Merkez noktası
            cv2.circle(display_frame, (center_x, center_y), 5, color, -1)
            
            # Güven skoru
            cv2.putText(display_frame, f"Dart: {confidence:.2f}", 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Alan bilgisi
            cv2.putText(display_frame, f"Area: {dart['area']}", 
                       (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # İstatistik bilgileri
        stats_text = [
            f"Total: {self.total_detections}",
            f"Valid: {self.valid_detections}", 
            f"Threshold: {self.confidence_threshold}",
            f"Found: {len(detections)}"
        ]
        
        for i, text in enumerate(stats_text):
            cv2.putText(display_frame, text, (10, 30 + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return display_frame
    
    def get_detection_stats(self) -> Dict:
        """Tespit istatistiklerini döndürür"""
        
        return {
            'total_detections': self.total_detections,
            'valid_detections': self.valid_detections,
            'confidence_threshold': self.confidence_threshold,
            'last_detection_time': self.last_detection_time,
            'valid_ratio': (self.valid_detections / max(self.total_detections, 1)) * 100
        }
    
    def set_confidence_threshold(self, threshold: float):
        """Güven eşiğini günceller"""
        self.confidence_threshold = max(0.1, min(1.0, threshold))
        print(f"[DartDetector] Güven eşiği güncellendi: {self.confidence_threshold}")


def test_dart_detector():
    """Test fonksiyonu - bağımsız çalışabilir"""
    
    print("=" * 50)
    print("🎯 DART DETECTOR TEST")
    print("=" * 50)
    
    try:
        # Detector oluştur
        detector = DartDetector(confidence_threshold=0.5)
        
        # Kamera başlat
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("❌ Kamera açılamadı!")
            return
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("\n🚀 Dart tespit testi başlıyor...")
        print("📋 Kontroller:")
        print("  'q' - Çıkış")
        print("  '+' - Güven eşiği artır")
        print("  '-' - Güven eşiği azalt")
        print("  's' - İstatistikleri göster")
        
        previous_center = None
        
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            
            # Dart tespiti
            detections = detector.detect_darts(frame)
            
            # En iyi dart'ı seç
            best_dart = detector.get_best_dart(detections)
            stable_dart = detector.get_stable_dart(detections, previous_center)
            
            # Kararlı dart varsa merkezi güncelle
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
