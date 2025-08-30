#!/usr/bin/env python3
"""
BARLAS Dart Laser Targeting System Test
Gerçek Arduino Pan-Tilt + Lazer sistemi ile dart hedefleme testi
"""

import cv2
import numpy as np
import time
from targeting_system import DartLaserTargetingSystem
import argparse

def test_targeting_system():
    """Dart Laser Targeting sistemini test eder"""
    
    print("🎯 BARLAS Dart Laser Targeting System")
    print("=" * 50)
    
    # Arduino port'unu belirle (varsayılan: COM3)
    arduino_port = "COM3"
    
    # Kullanıcıdan port bilgisi al
    user_port = input(f"Arduino port (varsayılan {arduino_port}): ").strip()
    if user_port:
        arduino_port = user_port
    
    try:
        # Targeting sistemi başlat
        print(f"🚀 Targeting sistem başlatılıyor - Arduino: {arduino_port}")
        targeting_system = DartLaserTargetingSystem(arduino_port)
        
        print("✅ Targeting sistem hazır!")
        print()
        print("📋 Sistem Özellikleri:")
        print(f"  - YOLO + Hough Circle dart detection")
        print(f"  - Arduino pan-tilt control ({arduino_port})")
        print(f"  - Lazer pulse targeting")
        print(f"  - Hedef kilit süresi: {targeting_system.target_lock_time}s")
        print(f"  - Güven eşiği: {targeting_system.target_confidence_threshold}")
        print()
        print("🎮 Kontroller:")
        print("  SPACE: Manuel lazer ateşleme")
        print("  C: Kalibasyon modu")
        print("  S: Servo test")
        print("  ESC: Çıkış")
        print("=" * 50)
        
        # Test başlat
        input("ENTER tuşuna basarak başlatın...")
        targeting_system.start_targeting()
        
    except KeyboardInterrupt:
        print("\n🛑 Test kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"\n❌ Hata: {e}")
        print("💡 Çözüm önerileri:")
        print("  - Arduino'nun doğru port'a bağlı olduğunu kontrol edin")
        print("  - Arduino kodunun yüklendiğini doğrulayın") 
        print("  - Kamera bağlantısını kontrol edin")
    
    print("\n✅ Test tamamlandı!")

def test_arduino_only():
    """Sadece Arduino Pan-Tilt sistemini test eder"""
    
    print("🔧 Arduino Pan-Tilt Test Modu")
    print("=" * 30)
    
    arduino_port = input("Arduino port (örn: COM3): ").strip()
    if not arduino_port:
        print("❌ Port belirtilmedi!")
        return
    
    try:
        from laser_controller import LaserPanTiltController
        
        print(f"🔌 Arduino bağlantısı kuruluyor: {arduino_port}")
        controller = LaserPanTiltController(arduino_port)
        
        print("✅ Arduino bağlandı!")
        print()
        print("🎮 Test komutları:")
        print("  w/s: Tilt yukarı/aşağı")
        print("  a/d: Pan sol/sağ")
        print("  l: Lazer on/off")
        print("  h: Home position")
        print("  q: Çıkış")
        
        while True:
            cmd = input("\nKomut: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'w':
                controller.move_tilt_relative(5)
                print("↑ Tilt yukarı")
            elif cmd == 's':
                controller.move_tilt_relative(-5)
                print("↓ Tilt aşağı")
            elif cmd == 'a':
                controller.move_pan_relative(-5)
                print("← Pan sol")
            elif cmd == 'd':
                controller.move_pan_relative(5)
                print("→ Pan sağ")
            elif cmd == 'l':
                controller.toggle_laser()
                print("🔴 Lazer toggle")
            elif cmd == 'h':
                controller.go_home()
                print("🏠 Home position")
            else:
                print("❓ Bilinmeyen komut")
        
        controller.close()
        print("✅ Arduino bağlantısı kapatıldı")
        
    except Exception as e:
        print(f"❌ Arduino hatası: {e}")

def show_menu():
    """Ana menüyü gösterir"""
    
    print()
    print("🎯 BARLAS Dart Laser System Test")
    print("=" * 40)
    print("1. Tam Targeting Test (YOLO + Arduino)")
    print("2. Sadece Arduino Pan-Tilt Test")
    print("3. Sadece YOLO Detection Test")
    print("4. Çıkış")
    print("=" * 40)
    
    choice = input("Seçiminiz (1-4): ").strip()
    return choice

if __name__ == "__main__":
    while True:
        choice = show_menu()
        
        if choice == '1':
            test_targeting_system()
        elif choice == '2':
            test_arduino_only()
        elif choice == '3':
            print("🔄 YOLO test başlatılıyor...")
            import subprocess
            subprocess.run(["python", "test_yolo_hough.py"])
        elif choice == '4':
            print("👋 Görüşmek üzere!")
            break
        else:
            print("❌ Geçersiz seçim!")
        
        input("\nDevam etmek için ENTER...")
