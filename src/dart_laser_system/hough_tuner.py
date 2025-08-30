"""
BARLAS Hough Circle Parameter Tuner
Gerçek zamanlı Hough Circle parametrelerini ayarlama aracı
"""
import cv2
import numpy as np
import sys
import os
import time
from dart_detector import DartDetector

class HoughCircleTuner:
    """Hough Circle parametrelerini gerçek zamanlı ayarlama"""
    
    def __init__(self):
        """Tuner başlat"""
        print("🔧 BARLAS Hough Circle Parameter Tuner")
        print("=" * 50)
        
        # Dart detector
        self.dart_detector = DartDetector(confidence_threshold=0.3)  # Düşük threshold - daha fazla test
        
        # Tunable parametreler
        self.params = {
            'param1': 60,           # Canny edge threshold
            'param2': 20,           # Accumulator threshold
            'minDist_ratio': 0.25,  # Min distance ratio
            'minRadius_ratio': 0.1, # Min radius ratio
            'blur_kernel': 7,       # Blur kernel (tek sayı)
            'blur_sigma': 15,       # Blur sigma (x10 for trackbar)
            'confidence': 30        # YOLO confidence (x100 for trackbar)
        }
        
        # Test frame'i için
        self.current_frame = None
        self.test_image_path = None
        
        print("✅ Tuner hazır!")
    
    def create_trackbars(self):
        """OpenCV trackbar'larını oluştur"""
        cv2.namedWindow('Hough Parameters', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Hough Parameters', 400, 300)
        
        # Trackbar'lar oluştur
        cv2.createTrackbar('Param1 (Edge)', 'Hough Parameters', self.params['param1'], 100, self.update_param1)
        cv2.createTrackbar('Param2 (Accum)', 'Hough Parameters', self.params['param2'], 50, self.update_param2)
        cv2.createTrackbar('MinDist x100', 'Hough Parameters', int(self.params['minDist_ratio']*100), 100, self.update_mindist)
        cv2.createTrackbar('MinRadius x100', 'Hough Parameters', int(self.params['minRadius_ratio']*100), 50, self.update_minradius)
        cv2.createTrackbar('Blur Kernel', 'Hough Parameters', self.params['blur_kernel'], 15, self.update_blur_kernel)
        cv2.createTrackbar('Blur Sigma x10', 'Hough Parameters', self.params['blur_sigma'], 50, self.update_blur_sigma)
        cv2.createTrackbar('YOLO Conf x100', 'Hough Parameters', self.params['confidence'], 100, self.update_confidence)
    
    def update_param1(self, val):
        self.params['param1'] = max(10, val)
        self.update_hough_params()
    
    def update_param2(self, val):
        self.params['param2'] = max(5, val)
        self.update_hough_params()
    
    def update_mindist(self, val):
        self.params['minDist_ratio'] = val / 100.0
        self.update_hough_params()
    
    def update_minradius(self, val):
        self.params['minRadius_ratio'] = val / 100.0
        self.update_hough_params()
    
    def update_blur_kernel(self, val):
        # Tek sayı olmalı
        kernel = max(3, val)
        if kernel % 2 == 0:
            kernel += 1
        self.params['blur_kernel'] = kernel
        self.update_hough_params()
    
    def update_blur_sigma(self, val):
        self.params['blur_sigma'] = val
        self.update_hough_params()
    
    def update_confidence(self, val):
        self.params['confidence'] = val
        confidence_threshold = val / 100.0
        self.dart_detector.set_confidence_threshold(confidence_threshold)
    
    def update_hough_params(self):
        """Hough parametrelerini güncelle"""
        self.dart_detector.set_hough_params(
            param1=self.params['param1'],
            param2=self.params['param2'],
            minDist_ratio=self.params['minDist_ratio'],
            minRadius_ratio=self.params['minRadius_ratio'],
            blur_kernel=self.params['blur_kernel'],
            blur_sigma=self.params['blur_sigma'] / 10.0
        )
    
    def start_live_tuning(self, use_camera=True, test_image_path=None):
        """Canlı ayarlama başlat"""
        
        if use_camera:
            # Kamera ile test
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("❌ Kamera açılamadı!")
                return
            
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print("📷 Kamera ile canlı tuning başlatıldı")
            
        elif test_image_path and os.path.exists(test_image_path):
            # Test görüntüsü ile
            print(f"🖼️ Test görüntüsü: {test_image_path}")
            cap = None
        else:
            print("❌ Geçerli input bulunamadı!")
            return
        
        # Trackbar'ları oluştur
        self.create_trackbars()
        
        print("🔧 Parametreleri trackbar'larla ayarlayın")
        print("📋 Kontroller:")
        print("  ESC: Çıkış")
        print("  S: Mevcut parametreleri kaydet")
        print("  R: Parametreleri resetle")
        
        frame_count = 0
        
        try:
            while True:
                if use_camera:
                    ret, frame = cap.read()
                    if not ret:
                        break
                else:
                    # Test görüntüsünü oku
                    frame = cv2.imread(test_image_path)
                    if frame is None:
                        print("❌ Test görüntüsü okunamadı!")
                        break
                
                frame_count += 1
                self.current_frame = frame.copy()
                
                # Dart detection uygula
                start_time = time.time()
                
                # YOLO detection
                detections = self.dart_detector.detect_darts(frame)
                
                # En iyi dart'ı bul (Hough Circle ile)
                best_dart = None
                if detections:
                    best_dart = self.dart_detector.get_best_dart(detections)
                
                process_time = (time.time() - start_time) * 1000
                
                # Görselleştirme
                display_frame = frame.copy()
                
                # Detection sonuçları
                info_y = 30
                cv2.putText(display_frame, f"Frame: {frame_count}", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                info_y += 25
                
                cv2.putText(display_frame, f"Detections: {len(detections)}", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                info_y += 25
                
                cv2.putText(display_frame, f"Process: {process_time:.1f}ms", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                info_y += 25
                
                # Mevcut parametreler
                cv2.putText(display_frame, f"P1:{self.params['param1']} P2:{self.params['param2']}", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                info_y += 20
                
                cv2.putText(display_frame, f"Conf:{self.params['confidence']/100:.2f} Blur:{self.params['blur_kernel']}", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                # Tüm detection'ları çiz
                for detection in detections:
                    x, y, w, h = detection['bbox']
                    confidence = detection['confidence']
                    center = detection['center']
                    
                    # Renk seç
                    if detection == best_dart:
                        color = (0, 255, 0)      # Yeşil - en iyi
                        thickness = 3
                    elif confidence >= self.dart_detector.confidence_threshold:
                        color = (0, 255, 255)    # Sarı - geçerli
                        thickness = 2
                    else:
                        color = (0, 128, 255)    # Turuncu - düşük güven
                        thickness = 1
                    
                    # Bounding box
                    cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, thickness)
                    
                    # Merkez
                    cv2.circle(display_frame, center, 6, color, -1)
                    
                    # Güven değeri
                    cv2.putText(display_frame, f"{confidence:.2f}", 
                               (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Hough Circle bilgisi
                    if detection.get('refined', False):
                        radius = detection.get('hough_radius', 0)
                        cv2.circle(display_frame, center, radius, (255, 0, 255), 2)  # Magenta circle
                        cv2.putText(display_frame, f"R={radius}", 
                                   (x, y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                
                cv2.imshow('Dart Detection with Hough Tuning', display_frame)
                
                # Klavye kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
                elif key == ord('s'):  # Save
                    self.save_parameters()
                elif key == ord('r'):  # Reset
                    self.reset_parameters()
                
                # Test görüntüsü modunda bekle
                if not use_camera:
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\\n⏹️ Kullanıcı tarafından durduruldu")
        finally:
            if use_camera and cap:
                cap.release()
            cv2.destroyAllWindows()
    
    def save_parameters(self):
        """Mevcut parametreleri kaydet"""
        print("💾 Parametreler kaydediliyor...")
        print("📋 Optimize edilmiş Hough parametreleri:")
        for key, value in self.params.items():
            if key in ['minDist_ratio', 'minRadius_ratio']:
                print(f"  {key}: {value:.3f}")
            elif key in ['blur_sigma']:
                print(f"  {key}: {value/10:.2f}")
            elif key == 'confidence':
                print(f"  {key}_threshold: {value/100:.2f}")
            else:
                print(f"  {key}: {value}")
    
    def reset_parameters(self):
        """Parametreleri varsayılana sıfırla"""
        print("🔄 Parametreler sıfırlanıyor...")
        self.params = {
            'param1': 60,
            'param2': 20,
            'minDist_ratio': 0.25,
            'minRadius_ratio': 0.1,
            'blur_kernel': 7,
            'blur_sigma': 15,  # x10
            'confidence': 30    # x100
        }
        self.update_hough_params()
        self.dart_detector.set_confidence_threshold(0.3)


def main():
    """Ana fonksiyon"""
    print("🔧 BARLAS Hough Circle Parameter Tuner")
    print("=" * 50)
    
    tuner = HoughCircleTuner()
    
    print("📋 Test modları:")
    print("1️⃣  Kamera ile canlı tuning")
    print("2️⃣  Test görüntüsü ile tuning")
    print("0️⃣  Çıkış")
    
    while True:
        choice = input("\\n🚀 Seçiminizi yapın (0-2): ").strip()
        
        if choice == "1":
            print("📷 Kamera ile canlı tuning başlatılıyor...")
            tuner.start_live_tuning(use_camera=True)
            break
        
        elif choice == "2":
            # Test görüntüsü yolu sor
            print("🖼️ Test görüntüsü modları:")
            print("  a) Dataset'ten örnek kullan")
            print("  b) Kendi görüntüm")
            
            img_choice = input("Görüntü seçimi (a/b): ").strip().lower()
            
            if img_choice == 'a':
                # Dataset'ten örnek bul
                dataset_path = "../../dataset/raw_images"
                if os.path.exists(dataset_path):
                    import glob
                    images = glob.glob(os.path.join(dataset_path, "*.jpg")) + glob.glob(os.path.join(dataset_path, "*.png"))
                    if images:
                        test_image = images[0]
                        print(f"📁 Dataset görüntüsü: {test_image}")
                        tuner.start_live_tuning(use_camera=False, test_image_path=test_image)
                    else:
                        print("❌ Dataset'te görüntü bulunamadı!")
                else:
                    print("❌ Dataset klasörü bulunamadı!")
            elif img_choice == 'b':
                img_path = input("Görüntü yolu girin: ").strip()
                if os.path.exists(img_path):
                    tuner.start_live_tuning(use_camera=False, test_image_path=img_path)
                else:
                    print("❌ Görüntü bulunamadı!")
            break
        
        elif choice == "0":
            print("👋 Çıkılıyor...")
            break
        
        else:
            print("❌ Geçersiz seçim!")


if __name__ == "__main__":
    main()
