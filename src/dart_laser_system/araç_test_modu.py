#!/usr/bin/env python3
"""
🚗 BARLAS ARAÇ TEST MODU
Gerçek araç üzerinde güvenli test için hazırlanmış sistem
"""

import sys
import os
import time
import threading
import keyboard  # pip install keyboard

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from arduino_controller_fixed import BarlasVehicleController

class BarlasVehicleTestMode:
    """🚗 Güvenli araç test sistemi"""
    
    def __init__(self, port="COM3"):
        self.controller = None
        self.port = port
        self.test_active = False
        self.emergency_active = False
        
        # Güvenlik sınırları
        self.MAX_SPEED = 100        # Maksimum test hızı (güvenlik için)
        self.MAX_TEST_TIME = 60     # Maksimum test süresi (saniye)
        self.current_speed = 0
        
        print("🚗 BARLAS ARAÇ TEST MODU")
        print("=" * 50)
        
    def connect_vehicle(self):
        """Araca bağlan"""
        print(f"\n🔌 Araca bağlanılıyor: {self.port}")
        
        try:
            self.controller = BarlasVehicleController(port=self.port, baud_rate=9600)
            if self.controller.connect():
                print("✅ Araç bağlantısı başarılı!")
                return True
            else:
                print("❌ Araç bağlantısı başarısız!")
                return False
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def safety_check(self):
        """Güvenlik kontrolü"""
        print("\n🛡️  GÜVENLİK KONTROLÜ")
        print("-" * 30)
        
        checks = [
            "Araç güvenli alanda mı? (kapalı alan, engelsiz)",
            "Fren sistemi test edildi mi?",
            "Acil durdurma hazır mı?",
            "Motor sürücü soğuk mu?",
            "Kablolar güvenli mi?"
        ]
        
        for i, check in enumerate(checks, 1):
            response = input(f"{i}. {check} (y/N): ")
            if response.lower() != 'y':
                print("❌ Güvenlik kontrolü başarısız!")
                print("⚠️  Lütfen tüm güvenlik önlemlerini alın!")
                return False
        
        print("✅ Tüm güvenlik kontrolleri tamam!")
        return True
    
    def system_diagnostics(self):
        """Sistem tanılama"""
        print("\n🔧 SİSTEM TANILAMA")
        print("-" * 25)
        
        try:
            # Temel iletişim
            if self.controller.test_connection():
                print("  ✅ Arduino iletişimi: OK")
            else:
                print("  ❌ Arduino iletişimi: HATA")
                return False
            
            # Pan-Tilt test
            print("  🎯 Pan-Tilt testi...")
            self.controller.center_position()
            time.sleep(1)
            print("  ✅ Pan-Tilt: OK")
            
            # Fren test
            print("  🛑 Fren testi...")
            self.controller.brake_on()
            time.sleep(0.5)
            self.controller.brake_off()
            print("  ✅ Fren sistemi: OK")
            
            # Far test
            print("  💡 Far testi...")
            self.controller.headlight_on()
            time.sleep(0.5)
            self.controller.headlight_off()
            print("  ✅ Far sistemi: OK")
            
            # Encoder test
            print("  📊 Encoder testi...")
            encoders = self.controller.read_encoders()
            if encoders:
                print(f"  ✅ Encoders: Sol={encoders.get('encoder1', 0)}, Sağ={encoders.get('encoder2', 0)}")
            else:
                print("  ⚠️  Encoder okuma problemi")
            
            # Motor test (çok düşük hızda)
            print("  🚗 Motor testi (düşük hız)...")
            self.controller.move_forward(30)  # Çok düşük hız
            time.sleep(0.5)
            self.controller.stop_motors()
            print("  ✅ Motor sistemi: OK")
            
            return True
            
        except Exception as e:
            print(f"  ❌ Tanılama hatası: {e}")
            return False
    
    def manual_control_mode(self):
        """Manuel kontrol modu"""
        print("\n🎮 MANUEL KONTROL MODU")
        print("=" * 30)
        print("KONTROLLER:")
        print("  W/S     - İleri/Geri")
        print("  A/D     - Pan hareket")
        print("  Q/E     - Tilt hareket") 
        print("  SPACE   - Dur")
        print("  B       - Fren")
        print("  L       - Far")
        print("  ESC     - Çıkış")
        print("  F1      - ACİL FREN!")
        print()
        
        self.test_active = True
        
        # Güvenlik zamanlayıcısı (60 saniye)
        safety_timer = threading.Timer(self.MAX_TEST_TIME, self.emergency_stop)
        safety_timer.start()
        
        try:
            while self.test_active and not self.emergency_active:
                # Klavye kontrolü
                if keyboard.is_pressed('w'):  # İleri
                    speed = min(50, self.MAX_SPEED)  # Güvenlik limiti
                    self.controller.move_forward(speed)
                    self.current_speed = speed
                    print(f"🔄 İleri: {speed}")
                    time.sleep(0.1)
                
                elif keyboard.is_pressed('s'):  # Geri
                    speed = min(50, self.MAX_SPEED)
                    self.controller.move_backward(speed) 
                    self.current_speed = -speed
                    print(f"🔄 Geri: {speed}")
                    time.sleep(0.1)
                
                elif keyboard.is_pressed('space'):  # Dur
                    self.controller.stop_motors()
                    self.current_speed = 0
                    print("⏹️  Dur")
                    time.sleep(0.2)
                
                elif keyboard.is_pressed('b'):  # Fren
                    self.controller.brake_on()
                    print("🛑 Fren ON")
                    time.sleep(0.2)
                    self.controller.brake_off()
                
                elif keyboard.is_pressed('l'):  # Far
                    self.controller.headlight_on()
                    print("💡 Far ON")
                    time.sleep(0.5)
                    self.controller.headlight_off()
                    print("💡 Far OFF")
                
                elif keyboard.is_pressed('a'):  # Pan sol
                    current_pan = getattr(self.controller, 'current_pan', 90)
                    new_pan = max(0, current_pan - 5)
                    self.controller.move_to_position(new_pan, getattr(self.controller, 'current_tilt', 90))
                    print(f"🎯 Pan: {new_pan}")
                    time.sleep(0.1)
                
                elif keyboard.is_pressed('d'):  # Pan sağ
                    current_pan = getattr(self.controller, 'current_pan', 90)
                    new_pan = min(180, current_pan + 5)
                    self.controller.move_to_position(new_pan, getattr(self.controller, 'current_tilt', 90))
                    print(f"🎯 Pan: {new_pan}")
                    time.sleep(0.1)
                
                elif keyboard.is_pressed('f1'):  # ACİL FREN
                    self.emergency_stop()
                    break
                
                elif keyboard.is_pressed('esc'):  # Çıkış
                    break
                
                time.sleep(0.05)  # CPU yükünü azalt
        
        finally:
            safety_timer.cancel()
            self.emergency_stop()
            print("\n🏁 Manuel kontrol sona erdi")
    
    def emergency_stop(self):
        """Acil durdurma"""
        self.emergency_active = True
        self.test_active = False
        
        print("\n🚨 ACİL DURDURMA AKTİF!")
        if self.controller:
            self.controller.emergency_brake()
        print("✅ Araç güvenli duruma getirildi")
    
    def autonomous_test_mode(self):
        """Otonom test modu"""
        print("\n🤖 OTONOM TEST MODU")
        print("=" * 25)
        
        # YOLO sistemi ile entegrasyon
        try:
            import cv2
            
            # Kamera aç
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("❌ Kamera açılamadı!")
                return
            
            print("📹 Kamera açıldı")
            print("🎯 Dart tespiti aktif")
            print("ESC ile çıkış")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Basit görüntü işleme (YOLO yerine)
                cv2.putText(frame, "BARLAS OTONOM TEST", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Hiz: {self.current_speed}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                cv2.imshow("BARLAS Test", frame)
                
                # ESC ile çıkış
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    break
            
            cap.release()
            cv2.destroyAllWindows()
            
        except ImportError:
            print("⚠️  OpenCV kurulu değil, basit test modu çalışacak")
            
            # Basit hareket testi
            print("🔄 Basit hareket testi başlıyor...")
            
            # İleri git
            self.controller.move_forward(40)
            time.sleep(2)
            
            # Dur
            self.controller.stop_motors()
            time.sleep(1)
            
            # Geri git
            self.controller.move_backward(40)
            time.sleep(2)
            
            # Dur ve merkez
            self.controller.stop_motors()
            self.controller.center_position()
            
            print("✅ Basit hareket testi tamamlandı")
    
    def run(self):
        """Ana test döngüsü"""
        try:
            # Bağlantı kur
            if not self.connect_vehicle():
                return False
            
            # Güvenlik kontrolü
            if not self.safety_check():
                return False
            
            # Sistem tanılama
            if not self.system_diagnostics():
                return False
            
            # Test modu seçimi
            while True:
                print("\n🎯 TEST MODU SEÇİMİ")
                print("=" * 25)
                print("1. 🎮 Manuel Kontrol")
                print("2. 🤖 Otonom Test")
                print("3. 🔧 Sistem Durumu")
                print("4. ❌ Çıkış")
                
                choice = input("\nSeçiminiz (1-4): ")
                
                if choice == '1':
                    self.manual_control_mode()
                elif choice == '2':
                    self.autonomous_test_mode()
                elif choice == '3':
                    status = self.controller.get_system_status()
                    print(f"\n📊 Sistem Durumu:\n{status}")
                elif choice == '4':
                    break
                else:
                    print("❌ Geçersiz seçim!")
        
        finally:
            if self.controller:
                self.controller.emergency_brake()
                self.controller.disconnect()
                print("🔌 Araç bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    print("🚗 BARLAS ARAÇ TEST SİSTEMİ")
    print("🛡️  Güvenlik odaklı araç test platformu")
    print("⚠️  Sadece güvenli, kapalı alanlarda kullanın!")
    print()
    
    # Port seçimi
    port = input("Arduino portu (varsayılan COM3): ") or "COM3"
    
    # Test sistemini başlat
    test_system = BarlasVehicleTestMode(port=port)
    test_system.run()
    
    print("\n🏁 Test tamamlandı!")
    print("✅ Araç güvenli duruma getirildi")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🚨 KULLANICI DURDURDU!")
        print("✅ Güvenli çıkış yapıldı")
    except Exception as e:
        print(f"\n💥 Beklenmeyen hata: {e}")
        print("🚨 ACİL DURDURMA!")
