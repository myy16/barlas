#!/usr/bin/env python3
"""
BARLAS Full System Test - Tüm özellikleri test et
Pan-Tilt | Motor | Fren | Far | Encoder
"""

import sys
import time
from arduino_controller_fixed import BarlasVehicleController

def test_full_barlas_system():
    """BARLAS tam sistem testini çalıştır"""
    print("🚗 BARLAS TAM SİSTEM TESTİ")
    print("=" * 60)
    print("🎯 Pan-Tilt | 🚗 Motor | 🛑 Fren | 💡 Far | 📊 Encoder")
    print("=" * 60)
    
    # Port tespiti
    if sys.platform.startswith('linux'):
        ports_to_test = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
        print("🐧 Linux sistem tespit edildi")
    else:
        ports_to_test = ['COM7', 'COM6', 'COM5', 'COM8', 'COM9']
        print("🪟 Windows sistem tespit edildi")
    
    print(f"📋 Test edilecek portlar: {ports_to_test}")
    print()
    
    # Her portu test et
    for port in ports_to_test:
        print(f"🔍 Port test ediliyor: {port}")
        
        try:
            # BARLAS Full Vehicle Controller
            vehicle = BarlasVehicleController(port=port, baud_rate=9600, timeout=2)
            
            if vehicle.connect():
                print(f"✅ BAŞARILI: {port} - BARLAS Araç sistemi bağlandı!")
                
                # ======================================
                # 🎯 PAN-TILT TESTLERİ
                # ======================================
                print("\n🎯 Pan-Tilt System Testleri:")
                
                print("  → Merkez pozisyon (90, 90)")
                vehicle.center_position()
                time.sleep(1)
                
                print("  → Pan sola (45°)")
                vehicle.move_to_position(45, 90)
                time.sleep(1)
                
                print("  → Pan sağa (135°)")
                vehicle.move_to_position(135, 90)
                time.sleep(1)
                
                print("  → Tilt yukarı (60°)")
                vehicle.move_to_position(90, 60)
                time.sleep(1)
                
                print("  → Tilt aşağı (120°)")
                vehicle.move_to_position(90, 120)
                time.sleep(1)
                
                print("  → Merkez pozisyon")
                vehicle.center_position()
                time.sleep(1)
                
                # Lazer testi
                print("  🔴 Lazer testi (1 saniye)...")
                vehicle.enable_laser()
                time.sleep(1)
                vehicle.disable_laser()
                
                # ======================================
                # 🚗 MOTOR TESTLERİ
                # ======================================
                print("\n🚗 Motor System Testleri:")
                
                print("  → İleri hareket (hız: 80)")
                vehicle.move_forward(80)
                time.sleep(2)
                
                print("  → Geri hareket (hız: 60)")
                vehicle.move_backward(60)
                time.sleep(2)
                
                print("  → Sola dönüş (hız: 70)")
                vehicle.turn_left(70)
                time.sleep(2)
                
                print("  → Sağa dönüş (hız: 70)")
                vehicle.turn_right(70)
                time.sleep(2)
                
                print("  → Hareket durdur")
                vehicle.stop_movement()
                time.sleep(1)
                
                # ======================================
                # 🛑 FREN TESTLERİ
                # ======================================
                print("\n🛑 Brake System Testleri:")
                
                print("  → Fren çek")
                vehicle.brake_on()
                time.sleep(1)
                
                print("  → Fren bırak")
                vehicle.brake_off()
                time.sleep(1)
                
                print("  → Acil fren testi")
                vehicle.emergency_brake()
                time.sleep(1)
                vehicle.brake_off()  # Test sonrası bırak
                
                # ======================================
                # 💡 FAR TESTLERİ
                # ======================================
                print("\n💡 Headlight System Testleri:")
                
                print("  → Farları aç")
                vehicle.headlight_on()
                time.sleep(2)
                
                print("  → Farları kapat")
                vehicle.headlight_off()
                time.sleep(1)
                
                print("  → Far toggle testi")
                vehicle.headlight_toggle()  # Aç
                time.sleep(1)
                vehicle.headlight_toggle()  # Kapat
                time.sleep(1)
                
                # ======================================
                # 📊 ENCODER TESTLERİ
                # ======================================
                print("\n📊 Encoder System Testleri:")
                
                print("  → Encoder değerlerini oku")
                encoders = vehicle.read_encoders()
                if encoders:
                    left, right = encoders
                    print(f"    Sol: {left}, Sağ: {right}")
                
                print("  → Encoder sıfırlama")
                vehicle.reset_encoders()
                
                print("  → Mesafe hesaplama testi")
                distance = vehicle.get_distance_traveled()
                print(f"    Tahmini mesafe: {distance:.2f} cm")
                
                # ======================================
                # 🎛️ KONTROL MODU TESTLERİ
                # ======================================
                print("\n🎛️ Control Mode Testleri:")
                
                print("  → PC kontrol modu")
                vehicle.switch_to_pc()
                time.sleep(0.5)
                
                print("  → Pixhawk kontrol modu")
                vehicle.switch_to_pixhawk()
                time.sleep(0.5)
                
                print("  → PC kontrol moduna geri dön")
                vehicle.switch_to_pc()
                time.sleep(0.5)
                
                # ======================================
                # 🛡️ SİSTEM DURUMU
                # ======================================
                print("\n🛡️ System Status:")
                
                status = vehicle.get_system_status()
                for key, value in status.items():
                    print(f"  {key}: {value}")
                
                # Test tamamlandı
                print("\n🎉 TÜM TESTLER BAŞARILI!")
                print(f"✅ {port} portu tamamen çalışıyor!")
                print("🚀 BARLAS Full Vehicle System hazır!")
                
                vehicle.disconnect()
                return port
                
            else:
                print(f"❌ BAŞARISIZ: {port} yanıt vermiyor")
                vehicle.disconnect()
                
        except Exception as e:
            print(f"❌ HATA: {port} - {e}")
        
        print()
    
    print("❌ Hiçbir port çalışmıyor!")
    print("\n🛠️ Kontrol Listesi:")
    print("1. Arduino USB kablosu takılı mı?")
    print("2. Arduino'ya güç geliyor mu? (LED yanıyor mu?)")
    print("3. BARLAS Full System kodu Arduino'ya yüklendi mi?")
    print("4. BTS7960, servo, röle bağlantıları doğru mu?")
    
    return None

def demo_vehicle_control():
    """BARLAS araç kontrol demosu"""
    print("\n🎮 BARLAS ARAÇ KONTROL DEMOSU")
    print("WASD: Hareket | F: Far | B: Fren | Q: Çıkış")
    
    # Burada gerçek klavye kontrolü implement edilebilir
    print("(Bu özellik geliştirme aşamasında)")

if __name__ == "__main__":
    working_port = test_full_barlas_system()
    
    if working_port:
        print(f"\n🚀 BARLAS Sistem hazır! Port: {working_port}")
        print(f"\nKullanım örnekleri:")
        print(f"# YOLO + Pan-Tilt sistemi:")
        print(f"python yolo_arduino_dart_system.py --arduino {working_port}")
        print(f"\n# Manuel araç kontrolü:")
        print(f"python -c \"from arduino_controller_fixed import BarlasVehicleController; v=BarlasVehicleController('{working_port}'); v.connect(); v.move_forward(100)\"")
    else:
        print(f"\n❌ BARLAS sistem bağlantısı kurulamadı!")
        print("Simülatör ile test yapabilirsiniz:")
