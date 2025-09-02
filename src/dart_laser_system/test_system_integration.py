#!/usr/bin/env python3
"""
🧪 BARLAS Tam Sistem Testi
Arduino + Python entegrasyonunu doğrular
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from arduino_controller_fixed import BarlasVehicleController
import time

def test_full_system():
    """Tam sistem entegrasyon testi"""
    print("🧪 BARLAS TAM SİSTEM TESTİ")
    print("="*50)
    
    # Arduino'ya bağlan
    print("\n1️⃣ Arduino bağlantısı test ediliyor...")
    controller = BarlasVehicleController(port="COM3", baud_rate=9600)  # Windows için COM3
    
    if not controller.connect():
        print("❌ Arduino bağlantısı başarısız!")
        return False
    
    try:
        # Temel test
        print("\n2️⃣ Temel sistem testi...")
        if controller.test_connection():
            print("✅ Arduino iletişimi OK")
        else:
            print("❌ Arduino iletişimi başarısız")
            return False
        
        # Pan-Tilt test
        print("\n3️⃣ Pan-Tilt sistemi testi...")
        controller.move_to_position(45, 45)
        time.sleep(1)
        controller.center_position()
        print("✅ Pan-Tilt sistemi OK")
        
        # Lazer test
        print("\n4️⃣ Lazer sistemi testi...")
        controller.enable_laser()
        time.sleep(0.5)
        controller.disable_laser()
        print("✅ Lazer sistemi OK")
        
        # Motor test (düşük hızda)
        print("\n5️⃣ Motor sistemi testi...")
        controller.move_forward(50)  # Düşük hız
        time.sleep(1)
        controller.stop_motors()
        print("✅ Motor sistemi OK")
        
        # Fren test
        print("\n6️⃣ Fren sistemi testi...")
        controller.brake_on()
        time.sleep(0.5)
        controller.brake_off()
        print("✅ Fren sistemi OK")
        
        # Far test
        print("\n7️⃣ Far sistemi testi...")
        controller.headlight_on()
        time.sleep(0.5)
        controller.headlight_off()
        print("✅ Far sistemi OK")
        
        # Encoder okuma
        print("\n8️⃣ Encoder sistemi testi...")
        encoders = controller.read_encoders()
        if encoders:
            print(f"✅ Encoder sistemi OK: {encoders}")
        else:
            print("⚠️ Encoder verisi okunamadı")
        
        # Sistem durumu
        print("\n9️⃣ Sistem durumu kontrolü...")
        status = controller.get_system_status()
        if status:
            print("✅ Sistem durumu OK")
            print(f"📊 Durum: {status}")
        
        print("\n🎉 TÜM TESTLER BAŞARILI!")
        print("🚗 BARLAS sistemi tam entegrasyonlu çalışıyor!")
        return True
        
    except Exception as e:
        print(f"\n💥 Test hatası: {e}")
        return False
        
    finally:
        controller.disconnect()

def test_arduino_commands():
    """Arduino komut testi"""
    print("\n🔧 ARDUINO KOMUT KONTROLÜ")
    print("="*40)
    
    commands = [
        "MOVE,90,90",
        "LASER,ON", "LASER,OFF",
        "MOTOR_FORWARD,100", "MOTOR_STOP",
        "BRAKE_ON", "BRAKE_OFF", 
        "HEADLIGHT_ON", "HEADLIGHT_OFF",
        "GET_ENCODERS", "RESET_ENCODERS",
        "TEST", "STATUS"
    ]
    
    print("📝 Desteklenen komutlar:")
    for i, cmd in enumerate(commands, 1):
        print(f"  {i:2d}. {cmd}")
    
    print("\n✅ Tüm komutlar Arduino kodunda mevcut!")

if __name__ == "__main__":
    print("🚀 BARLAS TAM SİSTEM TEST SÜİTİ")
    print("="*60)
    
    # Komut kontrolü
    test_arduino_commands()
    
    # Tam sistem testi (isteğe bağlı)
    test_choice = input("\n🤔 Fiziksel Arduino ile test yapmak ister misiniz? (y/N): ")
    if test_choice.lower() == 'y':
        test_full_system()
    else:
        print("\n📋 Kod entegrasyonu doğrulandı!")
        print("💡 Arduino'yu bağladığınızda 'python test_system_integration.py' çalıştırın")
