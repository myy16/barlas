"""
BARLAS Dart Laser System Test Suite
Tüm modülleri test eder
"""
import time
import sys
import os

# Local import için path ayarla
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Modülleri import et
from laser_controller import LaserPanTiltController
from dart_detector import DartDetector
from targeting_system import DartLaserTargetingSystem

def test_imports():
    """Import testleri"""
    print("🔍 Import testleri...")
    
    try:
        from laser_controller import LaserPanTiltController
        print("  ✅ LaserPanTiltController")
        
        from dart_detector import DartDetector
        print("  ✅ DartDetector")
        
        from targeting_system import DartLaserTargetingSystem
        print("  ✅ DartLaserTargetingSystem")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Import hatası: {e}")
        return False

def test_laser_controller():
    """Lazer kontrol testi"""
    print("\n🔴 Lazer Controller Testi...")
    
    try:
        laser = LaserPanTiltController()
        
        print("  ✅ Controller oluşturuldu")
        
        # Temel hareketler
        laser.move_to_position(45, 60)
        time.sleep(1)
        
        laser.move_to_position(135, 120)
        time.sleep(1)
        
        laser.center_position()
        time.sleep(1)
        
        # Lazer testi
        laser.enable_laser()
        time.sleep(2)
        laser.disable_laser()
        
        print("  ✅ Hareket ve lazer testleri tamamlandı")
        
        laser.cleanup()
        return True
        
    except Exception as e:
        print(f"  ❌ Lazer controller hatası: {e}")
        return False

def test_dart_detector():
    """Dart detector testi"""
    print("\n🎯 Dart Detector Testi...")
    
    try:
        detector = DartDetector(confidence_threshold=0.5)
        print("  ✅ Detector oluşturuldu")
        
        # Dummy frame ile test
        import cv2
        import numpy as np
        
        # Test frame oluştur
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(test_frame, (320, 240), 50, (0, 255, 0), -1)
        
        detections = detector.detect_darts(test_frame)
        print(f"  ✅ Tespit sayısı: {len(detections)}")
        
        stats = detector.get_detection_stats()
        print(f"  ✅ İstatistikler alındı: {stats}")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Dart detector hatası: {e}")
        return False

def test_full_system():
    """Tam sistem testi"""
    print("\n🎯 Full System Testi...")
    
    try:
        system = DartLaserTargetingSystem(camera_index=0)
        print("  ✅ Sistem oluşturuldu")
        
        # Parametreleri ayarla
        system.set_targeting_parameters(
            confidence_threshold=0.6,
            lock_time=1.0,
            laser_duration=2.0
        )
        print("  ✅ Parametreler ayarlandı")
        
        # Kamera test etme (kısa süre)
        if system.initialize_camera():
            print("  ✅ Kamera başlatıldı")
            system.cap.release()
        else:
            print("  ⚠️ Kamera bulunamadı (normal)")
        
        system.cleanup()
        print("  ✅ Sistem temizlendi")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Full sistem hatası: {e}")
        return False

def interactive_test_menu():
    """Interaktif test menüsü"""
    
    while True:
        print("\n" + "="*50)
        print("🧪 BARLAS DART LASER SYSTEM TEST MENU")
        print("="*50)
        print("1. Import Testleri")
        print("2. Lazer Controller Test")
        print("3. Dart Detector Test (Kamera)")
        print("4. Full System Test")
        print("5. Lazer Controller Demo")
        print("6. Dart Detector Demo")
        print("7. Targeting System Demo")
        print("0. Çıkış")
        
        choice = input("\nSeçiminiz (0-7): ").strip()
        
        if choice == '0':
            print("👋 Test programından çıkılıyor...")
            break
            
        elif choice == '1':
            test_imports()
            
        elif choice == '2':
            test_laser_controller()
            
        elif choice == '3':
            if test_imports():
                from dart_detector import test_dart_detector
                test_dart_detector()
            
        elif choice == '4':
            test_full_system()
            
        elif choice == '5':
            if test_imports():
                from laser_controller import test_laser_controller
                test_laser_controller()
            
        elif choice == '6':
            if test_imports():
                from dart_detector import test_dart_detector
                test_dart_detector()
                
        elif choice == '7':
            if test_imports():
                from targeting_system import main
                main()
        
        else:
            print("❌ Geçersiz seçim!")
        
        input("\nDevam etmek için Enter tuşuna basın...")

def quick_system_test():
    """Hızlı sistem testi"""
    print("🚀 BARLAS Dart Laser System - Hızlı Test")
    print("="*50)
    
    # Tüm testleri sırayla çalıştır
    tests = [
        ("Import", test_imports),
        ("Laser Controller", test_laser_controller),
        ("Dart Detector", test_dart_detector),
        ("Full System", test_full_system)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n📋 {test_name} Testi çalışıyor...")
        
        try:
            result = test_func()
            results.append((test_name, result))
            
            if result:
                print(f"  ✅ {test_name}: BAŞARILI")
            else:
                print(f"  ❌ {test_name}: BAŞARISIZ")
                
        except Exception as e:
            print(f"  ❌ {test_name}: HATA - {e}")
            results.append((test_name, False))
    
    # Sonuçları göster
    print("\n" + "="*50)
    print("📊 TEST SONUÇLARI")
    print("="*50)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ BAŞARILI" if result else "❌ BAŞARISIZ"
        print(f"  {test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\nGenel Sonuç: {passed}/{total} test başarılı")
    
    if passed == total:
        print("🎉 Tüm testler başarılı! Sistem hazır.")
    else:
        print("⚠️ Bazı testler başarısız. Kontrol edin.")

def main():
    """Ana test fonksiyonu"""
    
    print("🧪 BARLAS Dart Laser System Test Suite")
    print("Hangi test türünü çalıştırmak istiyorsunuz?\n")
    print("1. Hızlı Test (Otomatik)")
    print("2. İnteraktif Test Menüsü")
    print("3. Sadece Import Test")
    
    choice = input("\nSeçiminiz (1-3): ").strip()
    
    if choice == '1':
        quick_system_test()
    elif choice == '2':
        interactive_test_menu()
    elif choice == '3':
        test_imports()
    else:
        print("Geçersiz seçim, hızlı test başlatılıyor...")
        quick_system_test()

if __name__ == "__main__":
    main()
