"""
BARLAS Dart Recognition Demo
Jupyter Notebook'tan modüler sisteme dönüştürülmüş dart tanıma sistemi
"""
import cv2
import time
from dart_recognize.yolo_predictions import YOLOPredictions

def demo_dart_detection():
    """Dart tanıma demo - webcam ile test"""
    
    print("=== BARLAS Dart Detection Demo ===")
    print("YOLO dart tanıma sistemi başlatılıyor...")
    
    try:
        # YOLO detector başlat
        yolo = YOLOPredictions()
        print("✅ YOLO modeli yüklendi")
        
        # Kamera başlat
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Kamera açılamadı! USB kamera bağlı mı?")
            return
        
        print("✅ Kamera başlatıldı")
        print("📹 Demo başlıyor - 'q' tuşu ile çıkış")
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ Kamera frame okunamadı")
                break
            
            frame_count += 1
            
            # Dart detection yap
            detections = yolo.get_detections(frame)
            
            # Sonuçları frame üzerine çiz
            result_frame = yolo.predictions(frame, draw_boxes=True)
            
            # Performans bilgisi
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time if elapsed_time > 0 else 0
            
            # Bilgi metni ekle
            info_text = f"FPS: {fps:.1f} | Dartlar: {len(detections)}"
            cv2.putText(result_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # En büyük dart'ı göster
            if detections:
                largest_dart = max(detections, key=lambda d: d['bbox'][2] * d['bbox'][3])
                x, y, w, h = largest_dart['bbox']
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Hedef merkezi
                cv2.circle(result_frame, (center_x, center_y), 15, (0, 255, 0), 3)
                cv2.putText(result_frame, "TARGET", (center_x-30, center_y-25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Tespit bilgisi
                confidence = largest_dart['confidence']
                print(f"[Demo] Dart tespit edildi - Güven: {confidence:.2f}, Merkez: ({center_x}, {center_y})")
            
            # Görüntüyü göster
            cv2.imshow('BARLAS Dart Detection Demo', result_frame)
            
            # Çıkış kontrolü
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Demo sonlandırılıyor...")
                break
            elif key == ord('s'):
                # Screenshot al
                filename = f"dart_detection_{frame_count}.jpg"
                cv2.imwrite(filename, result_frame)
                print(f"Screenshot kaydedildi: {filename}")
        
        # Temizlik
        cap.release()
        cv2.destroyAllWindows()
        
        print(f"✅ Demo tamamlandı - {frame_count} frame işlendi")
        print(f"📊 Ortalama FPS: {fps:.1f}")
        
    except Exception as e:
        print(f"❌ Demo hatası: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    demo_dart_detection()
