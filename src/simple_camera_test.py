"""
Basit Kamera Test - Web kamerasının çalışıp çalışmadığını kontrol eder
"""
import cv2
import time

def test_camera():
    print("=== Kamera Test Sistemi ===")
    
    # Farklı kamera indekslerini dene
    for camera_index in range(3):
        print(f"\nKamera {camera_index} deneniyor...")
        
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print(f"Kamera {camera_index}: AÇILAMADI")
            continue
        
        # Kamera özelliklerini ayarla
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Test frame'i oku
        ret, frame = cap.read()
        
        if ret:
            print(f"Kamera {camera_index}: BAŞARILI!")
            print(f"Frame boyutu: {frame.shape}")
            
            # 5 saniye görüntü göster
            print("5 saniye görüntü gösterilecek... (ESC ile çıkış)")
            
            start_time = time.time()
            while time.time() - start_time < 5:
                ret, frame = cap.read()
                if ret:
                    cv2.imshow(f'Kamera {camera_index} Test', frame)
                    
                    # ESC tuşu ile çıkış
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
                else:
                    print("Frame okunamadı!")
                    break
            
            cv2.destroyAllWindows()
            cap.release()
            return camera_index  # Çalışan kamerayı döndür
        else:
            print(f"Kamera {camera_index}: Frame okunamadı")
        
        cap.release()
    
    print("\nHiçbir kamera bulunamadı!")
    return None

if __name__ == "__main__":
    working_camera = test_camera()
    
    if working_camera is not None:
        print(f"\n✅ Çalışan kamera bulundu: Kamera {working_camera}")
        print("Bu kamera indeksini webcam_tabela_test.py'de kullanabilirsiniz")
    else:
        print("\n❌ Hiçbir kamera çalışmıyor")
        print("Kamera bağlantısını kontrol edin")
