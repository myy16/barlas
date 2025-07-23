"""
Kamera test scripti - hangi kamera indexinin çalıştığını kontrol eder
"""
import cv2

def test_camera():
    print("=== Kamera Test ===")
    
    # 0'dan 3'e kadar kamera indexlerini test et
    for i in range(4):
        print(f"\nKamera {i} test ediliyor...")
        cap = cv2.VideoCapture(i)
        
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"✅ Kamera {i} ÇALIŞIYOR - Çözünürlük: {frame.shape}")
                
                # 5 saniye görüntü göster
                cv2.imshow(f'Kamera {i}', frame)
                print(f"Kamera {i} görüntüsü gösteriliyor - 5 saniye sonra kapanacak")
                cv2.waitKey(5000)  # 5 saniye bekle
                cv2.destroyAllWindows()
            else:
                print(f"❌ Kamera {i} açıldı ama görüntü alınamıyor")
        else:
            print(f"❌ Kamera {i} açılamıyor")
        
        cap.release()
    
    print("\n=== Test Tamamlandı ===")

if __name__ == "__main__":
    test_camera()
