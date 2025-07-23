"""
Gelişmiş Kamera Test - Farklı backend'lerle deneme
"""
import cv2
import time

def test_camera_backends():
    """
    Farklı OpenCV backend'lerle kamera test et
    """
    print("=== Gelişmiş Kamera Test ===")
    
    # Farklı backend'ler dene
    backends = [
        (cv2.CAP_DSHOW, "DirectShow (Windows)"),
        (cv2.CAP_MSMF, "Media Foundation (Windows)"),
        (cv2.CAP_ANY, "Otomatik"),
        (cv2.CAP_V4L2, "Video4Linux (Linux)"),
    ]
    
    working_cameras = []
    
    for backend, backend_name in backends:
        print(f"\n--- {backend_name} Backend Testi ---")
        
        for camera_index in range(3):
            print(f"Kamera {camera_index} deneniyor...")
            
            try:
                cap = cv2.VideoCapture(camera_index, backend)
                
                if not cap.isOpened():
                    print(f"  ❌ Kamera {camera_index} açılamadı")
                    continue
                
                # Özellikleri ayarla
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Frame oku
                success_count = 0
                for i in range(10):  # 10 frame dene
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        success_count += 1
                    time.sleep(0.1)
                
                if success_count > 5:  # Yarısından fazlası başarılı
                    print(f"  ✅ Kamera {camera_index}: BAŞARILI! ({success_count}/10 frame)")
                    working_cameras.append((camera_index, backend, backend_name))
                    
                    # Kısa test göster
                    print(f"  Frame boyutu: {frame.shape if frame is not None else 'Bilinmiyor'}")
                    
                    # 3 saniye test
                    print("  3 saniye test ediliyor...")
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        ret, frame = cap.read()
                        if ret:
                            cv2.imshow(f'Test: {backend_name} - Kamera {camera_index}', frame)
                            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                                break
                    
                    cv2.destroyAllWindows()
                else:
                    print(f"  ❌ Kamera {camera_index}: Frame okunamadı ({success_count}/10)")
                
                cap.release()
                
            except Exception as e:
                print(f"  ❌ Kamera {camera_index}: Hata - {e}")
    
    return working_cameras

def recommend_best_camera(working_cameras):
    """
    En iyi kamera konfigürasyonunu öner
    """
    if not working_cameras:
        print("\n❌ Çalışan kamera bulunamadı!")
        return None
    
    print(f"\n✅ {len(working_cameras)} çalışan kamera konfigürasyonu bulundu:")
    
    for i, (camera_idx, backend, backend_name) in enumerate(working_cameras):
        print(f"  {i+1}. Kamera {camera_idx} - {backend_name}")
    
    # DirectShow'u tercih et (Windows için en stabil)
    for camera_idx, backend, backend_name in working_cameras:
        if backend == cv2.CAP_DSHOW:
            print(f"\n🎯 Önerilen: Kamera {camera_idx} - {backend_name}")
            return (camera_idx, backend)
    
    # DirectShow yoksa ilkini al
    camera_idx, backend, backend_name = working_cameras[0]
    print(f"\n🎯 Önerilen: Kamera {camera_idx} - {backend_name}")
    return (camera_idx, backend)

if __name__ == "__main__":
    working_cameras = test_camera_backends()
    best_camera = recommend_best_camera(working_cameras)
    
    if best_camera:
        camera_idx, backend = best_camera
        print(f"\nKullanım için kod:")
        print(f"cap = cv2.VideoCapture({camera_idx}, {backend})")
        print(f"# Backend: {backend}")
    else:
        print("\nKamera sorunlarını kontrol edin:")
        print("1. Kamera fiziksel olarak bağlı mı?")
        print("2. Başka uygulama kamerayı kullanıyor mu?")
        print("3. Kamera izinleri var mı?")
        print("4. Kamera sürücüleri güncel mi?")
