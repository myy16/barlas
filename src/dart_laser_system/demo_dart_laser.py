"""
BARLAS Dart Laser System - Test ve Demo
Sistem bileşenlerini ayrı ayrı test etmek için
"""
import cv2
import time
import threading
from dart_laser_targeting import DartLaserTargetingSystem, LaserPanTiltController
import os
import sys

# BARLAS modüllerini import et
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dart_recognize.yolo_predictions import YOLOPredictions

class DartLaserDemo:
    """
    Dart Lazer sistemi demo ve test sınıfı
    """
    
    def __init__(self):
        print("🎯 BARLAS Dart Laser Demo Başlatılıyor...")
        
        self.yolo = None
        self.laser_pantilt = None
        self.dart_laser_system = None
    
    def test_yolo_detection(self):
        """YOLO dart detection testi"""
        print("\n=== YOLO Dart Detection Test ===")
        
        try:
            self.yolo = YOLOPredictions()
            print("✅ YOLO modeli yüklendi")
            
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("❌ Kamera açılamadı")
                return False
            
            print("📹 YOLO test başlıyor - 'q' ile çıkış")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue
                
                # Dart detection
                detections = self.yolo.get_detections(frame)
                
                # Sonuçları çiz
                result_frame = self.yolo.predictions(frame)
                
                # Bilgi göster
                info = f"Dartlar: {len(detections)}"
                cv2.putText(result_frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('YOLO Dart Test', result_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            cap.release()
            cv2.destroyAllWindows()
            print("✅ YOLO test tamamlandı")
            return True
            
        except Exception as e:
            print(f"❌ YOLO test hatası: {e}")
            return False
    
    def test_laser_pantilt(self):
        """Lazer Pan-Tilt test"""
        print("\n=== Laser Pan-Tilt Test ===")
        
        try:
            self.laser_pantilt = LaserPanTiltController()
            print("✅ Lazer Pan-Tilt başlatıldı")
            
            print("🔄 Servo hareket testi...")
            
            # Test pozisyonları
            test_positions = [
                (90, 90),   # Merkez
                (60, 60),   # Sol-aşağı
                (120, 60),  # Sağ-aşağı
                (120, 120), # Sağ-yukarı
                (60, 120),  # Sol-yukarı
                (90, 90)    # Merkez'e dön
            ]
            
            for i, (pan, tilt) in enumerate(test_positions):
                print(f"  Pozisyon {i+1}: Pan={pan}°, Tilt={tilt}°")
                self.laser_pantilt.move_to_position(pan, tilt)
                time.sleep(1.5)
            
            print("🔴 Lazer test...")
            self.laser_pantilt.enable_laser()
            time.sleep(2)
            self.laser_pantilt.disable_laser()
            
            print("✅ Lazer Pan-Tilt test tamamlandı")
            return True
            
        except Exception as e:
            print(f"❌ Pan-Tilt test hatası: {e}")
            return False
    
    def test_integrated_system(self):
        """Entegre sistem testi"""
        print("\n=== Entegre Dart Lazer Sistem Test ===")
        
        try:
            self.dart_laser_system = DartLaserTargetingSystem()
            print("✅ Entegre sistem yüklendi")
            
            # Test parametreleri
            self.dart_laser_system.set_targeting_parameters(
                confidence_threshold=0.4,  # Düşük eşik (test için)
                lock_time=1.0,            # Hızlı kilitlenme
                laser_duration=2.0        # Kısa lazer süresi
            )
            
            print("🚀 Entegre sistem başlatılıyor...")
            if self.dart_laser_system.start_targeting_system():
                print("✅ Sistem aktif - otomatik dart hedefleme çalışıyor")
                print("📋 'q' ile durdurabilirsiniz")
                
                # Sistem çalışırken bekle
                try:
                    while self.dart_laser_system.is_running:
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("\n⚠️ Test kullanıcı tarafından durduruldu")
                
                print("✅ Entegre sistem test tamamlandı")
                return True
            else:
                print("❌ Sistem başlatılamadı")
                return False
                
        except Exception as e:
            print(f"❌ Entegre sistem test hatası: {e}")
            return False
    
    def interactive_demo(self):
        """İnteraktif demo modu"""
        print("\n=== İnteraktif Demo Modu ===")
        
        while True:
            print("\n🎯 BARLAS Dart Laser Demo Menüsü:")
            print("1. YOLO Dart Detection Test")
            print("2. Laser Pan-Tilt Test")  
            print("3. Entegre Sistem Test")
            print("4. Manuel Lazer Kontrolü")
            print("5. Sistem Durumu")
            print("0. Çıkış")
            
            choice = input("\nSeçiminiz (0-5): ").strip()
            
            if choice == '1':
                self.test_yolo_detection()
            elif choice == '2':
                self.test_laser_pantilt()
            elif choice == '3':
                self.test_integrated_system()
            elif choice == '4':
                self.manual_laser_control()
            elif choice == '5':
                self.show_system_status()
            elif choice == '0':
                print("Demo sonlandırılıyor...")
                break
            else:
                print("❌ Geçersiz seçim!")
    
    def manual_laser_control(self):
        """Manuel lazer kontrolü"""
        print("\n=== Manuel Lazer Kontrolü ===")
        
        if not self.laser_pantilt:
            self.laser_pantilt = LaserPanTiltController()
        
        print("Komutlar:")
        print("  w/s - Tilt yukarı/aşağı")
        print("  a/d - Pan sol/sağ")
        print("  l - Lazer aç/kapat")
        print("  c - Merkez pozisyon")
        print("  q - Çıkış")
        
        laser_on = False
        
        while True:
            command = input("Komut: ").lower().strip()
            
            if command == 'q':
                break
            elif command == 'w':
                self.laser_pantilt.adjust_tilt(5)
            elif command == 's':
                self.laser_pantilt.adjust_tilt(-5)
            elif command == 'a':
                self.laser_pantilt.adjust_pan(-5)
            elif command == 'd':
                self.laser_pantilt.adjust_pan(5)
            elif command == 'l':
                if laser_on:
                    self.laser_pantilt.disable_laser()
                    laser_on = False
                else:
                    self.laser_pantilt.enable_laser()
                    laser_on = True
            elif command == 'c':
                self.laser_pantilt.center_position()
                laser_on = False
            else:
                print("❌ Bilinmeyen komut!")
        
        # Lazer'i kapat
        if laser_on:
            self.laser_pantilt.disable_laser()
    
    def show_system_status(self):
        """Sistem durumu göster"""
        print("\n=== Sistem Durumu ===")
        
        # YOLO durumu
        if self.yolo:
            print("✅ YOLO: Yüklü ve hazır")
        else:
            print("❌ YOLO: Yüklenmemiş")
        
        # Pan-Tilt durumu
        if self.laser_pantilt:
            pan, tilt = self.laser_pantilt.pan_position, self.laser_pantilt.tilt_position
            laser_status = "AKTİF" if self.laser_pantilt.laser_active else "KAPALI"
            print(f"✅ Pan-Tilt: Pan={pan:.1f}°, Tilt={tilt:.1f}°, Lazer={laser_status}")
        else:
            print("❌ Pan-Tilt: Başlatılmamış")
        
        # Entegre sistem durumu
        if self.dart_laser_system:
            running = "ÇALIŞIYOR" if self.dart_laser_system.is_running else "DURDU"
            targeting = "AKTİF" if self.dart_laser_system.is_targeting else "PASİF"
            print(f"✅ Entegre Sistem: {running}, Hedefleme={targeting}")
        else:
            print("❌ Entegre Sistem: Başlatılmamış")
    
    def cleanup(self):
        """Temizlik"""
        print("\n🧹 Sistem temizliği...")
        
        if self.dart_laser_system:
            self.dart_laser_system.cleanup()
        
        if self.laser_pantilt:
            self.laser_pantilt.cleanup()
        
        cv2.destroyAllWindows()
        print("✅ Temizlik tamamlandı")


def main():
    """Ana demo fonksiyonu"""
    print("🎯" * 30)
    print("🎯 BARLAS DART LASER SYSTEM DEMO 🎯")
    print("🎯" * 30)
    
    demo = DartLaserDemo()
    
    try:
        # İnteraktif demo başlat
        demo.interactive_demo()
        
    except KeyboardInterrupt:
        print("\n⚠️ Demo kullanıcı tarafından iptal edildi")
    
    except Exception as e:
        print(f"\n❌ Demo hatası: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        demo.cleanup()
        print("\n🏁 Demo sonlandırıldı")


if __name__ == "__main__":
    main()
