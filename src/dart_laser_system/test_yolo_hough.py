#!/usr/bin/env python3
"""
YOLO + Hough Circle Dart Testi
Test script for dart detection with YOLO and Hough Circle refinement
"""

import cv2
import numpy as np
from dart_detector import DartDetector
import time

def test_dart_detection():
    """Doğrudan kamera ile dart tanıma testi"""
    
    print("🎯 BARLAS DART LASER SYSTEM")
    print("=" * 40)
    
    # Kamera başlat
    print("📷 Kamera açılıyor...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Kamera açılamadı!")
        return
        
    print("✅ Kamera hazır!")
    
    # Şimdi dart detector'ü yükle
    print("🎯 YOLO sistemi yükleniyor...")
    dart_detector = DartDetector()
    print("✅ Dart algılama sistemi hazır!")
    
    # Kamera çözünürlüğü
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("🚀 Test başlıyor! ESC=Çıkış")
    print("-" * 30)
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Kameradan görüntü alınamadı!")
            break
            
        frame_count += 1
        start_time = time.time()
        
        # YOLO ile dart tespiti
        detections = dart_detector.detect_darts(frame)
        
        # En iyi dart'ı seç (Hough Circle ile)
        best_dart = dart_detector.get_best_dart(detections)
        
        process_time = time.time() - start_time
        
        # Görselleştirme
        display_frame = frame.copy()
        
        if detections:
            print(f"📊 Frame {frame_count}: {len(detections)} dart bulundu, işlem süresi: {process_time*1000:.1f}ms")
            
            # Tüm detections'ları açık yeşil ile çiz
            for detection in detections:
                x, y, w, h = detection['bbox']
                x1, y1, x2, y2 = x, y, x+w, y+h
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(display_frame, f"{detection['confidence']:.2f}", 
                           (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
            # En iyi dart'ı koyu yeşil ile vurgula
            if best_dart:
                x, y, w, h = best_dart['bbox']
                x1, y1, x2, y2 = x, y, x+w, y+h
                center_x, center_y = best_dart['center']
                
                # Koyu yeşil kutu
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 128, 0), 3)
                
                # Merkez noktası
                cv2.circle(display_frame, (center_x, center_y), 8, (0, 0, 255), -1)
                
                # Bilgi metni
                info_text = f"BEST: {best_dart['confidence']:.2f}"
                if best_dart.get('refined', False):
                    info_text += f" [Hough R={best_dart.get('hough_radius', '?')}]"
                
                cv2.putText(display_frame, info_text, (x1, y1-25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 0), 2)
                
                # Hough Circle bilgisi
                if best_dart.get('refined', False):
                    print(f"  🎯 EN İYİ DART: ({center_x}, {center_y}) Güven:{best_dart['confidence']:.2f} [Hough Düzeltmesi: R={best_dart.get('hough_radius', '?')}]")
                else:
                    print(f"  🎯 EN İYİ DART: ({center_x}, {center_y}) Güven:{best_dart['confidence']:.2f} [YOLO merkezi]")
        
        else:
            print(f"🔍 Frame {frame_count}: Dart bulunamadı, işlem süresi: {process_time*1000:.1f}ms")
        
        # FPS bilgisi ekle
        fps = 1.0 / process_time if process_time > 0 else 0
        cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Görüntüyü göster
        cv2.imshow('YOLO + Hough Circle Dart Testi', display_frame)
        
        # ESC tuşu ile çıkış
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
    
    # Temizlik
    cap.release()
    cv2.destroyAllWindows()
    print("✅ Test tamamlandı!")

if __name__ == "__main__":
    test_dart_detection()
