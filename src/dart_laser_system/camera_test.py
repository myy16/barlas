"""
BARLAS Kamera Test Aracı
USB kamera ve dahili kamera testi
"""
import cv2
import time

def test_cameras():
    """Mevcut kameraları test et"""
    print("🎥 BARLAS Kamera Test Aracı")
    print("=" * 50)
    
    available_cameras = []
    
    # 0-10 arası kamera indekslerini test et
    for i in range(10):
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    height, width = frame.shape[:2]
                    available_cameras.append({
                        'index': i,
                        'width': width,
                        'height': height,
                        'type': 'Dahili Kamera' if i == 0 else f'USB Kamera {i}'
                    })
                    print(f"✅ Kamera {i}: {width}x{height} - {'Dahili' if i == 0 else 'USB'}")
                cap.release()
            else:
                cap.release()
        except Exception as e:
            pass
    
    if not available_cameras:
        print("❌ Hiç kamera bulunamadı!")
        return None
    
    print("\n📋 Bulunan Kameralar:")
    for cam in available_cameras:
        print(f"  {cam['index']}: {cam['type']} - {cam['width']}x{cam['height']}")
    
    # Kullanıcıdan seçim al
    print(f"\n🎯 Varsayılan: Kamera {available_cameras[0]['index']}")
    print("Farklı kamera seçmek için index girin, Enter'a basın...")
    
    try:
        choice = input("Kamera seçimi (0-9): ").strip()
        if choice:
            selected = int(choice)
            if any(cam['index'] == selected for cam in available_cameras):
                print(f"✅ Seçildi: Kamera {selected}")
                return selected
            else:
                print(f"⚠️ Kamera {selected} bulunamadı, varsayılan kullanılıyor")
    except:
        pass
    
    return available_cameras[0]['index']

def preview_camera(camera_index):
    """Kamera önizlemesi göster"""
    print(f"\n📹 Kamera {camera_index} Önizlemesi")
    print("ESC tuşuna basarak çıkın...")
    
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"❌ Kamera {camera_index} açılamadı!")
        return False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Bilgi yazısı
        cv2.putText(frame, f"Kamera {camera_index} - ESC: Cikis", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow(f'BARLAS Kamera {camera_index} Test', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return True

if __name__ == "__main__":
    selected_camera = test_cameras()
    if selected_camera is not None:
        preview_camera(selected_camera)
        print(f"\n🎯 Dart sistemi için kamera {selected_camera} kullanılacak")
        print(f"python yolo_arduino_dart_system.py --camera {selected_camera}")
