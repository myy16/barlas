#!/usr/bin/env python3
"""
Arduino Bağlantı Testi - Pan-Tilt Sistemini Test Et
"""

import sys
import time
from arduino_controller_simple import ArduinoPanTiltController

def test_arduino_connection():
    """Arduino bağlantısını test et"""
    print("🎯 BARLAS Arduino Pan-Tilt Bağlantı Testi")
    print("=" * 50)
    
    # İşletim sistemi kontrolü
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
            arduino = ArduinoPanTiltController(port=port, baud_rate=9600, timeout=2)
            
            if arduino.connect():
                print(f"✅ BAŞARILI: {port} portuna bağlandı!")
                
                # Temel hareket testleri
                print("🎮 Hareket testleri yapılıyor...")
                
                print("  → Merkez pozisyon (90, 90)")
                arduino.center_position()
                time.sleep(1)
                
                print("  → Pan sola (45°)")
                arduino.move_to_position(45, 90)
                time.sleep(1)
                
                print("  → Pan sağa (135°)")
                arduino.move_to_position(135, 90)
                time.sleep(1)
                
                print("  → Tilt yukarı (60°)")
                arduino.move_to_position(90, 60)
                time.sleep(1)
                
                print("  → Tilt aşağı (120°)")
                arduino.move_to_position(90, 120)
                time.sleep(1)
                
                print("  → Merkez pozisyon")
                arduino.center_position()
                time.sleep(1)
                
                # Lazer testi
                print("🔴 Lazer testi (1 saniye)...")
                arduino.enable_laser()
                time.sleep(1)
                arduino.disable_laser()
                
                arduino.disconnect()
                print(f"🎉 {port} portu TAMAMEN ÇALIŞIYOR!")
                return port
                
            else:
                print(f"❌ BAŞARISIZ: {port} yanıt vermiyor")
                arduino.disconnect()
                
        except Exception as e:
            print(f"❌ HATA: {port} - {e}")
        
        print()
    
    print("❌ Hiçbir port çalışmıyor!")
    print("\n🛠️ Kontrol Listesi:")
    print("1. Arduino USB kablosu takılı mı?")
    print("2. Arduino'ya güç geliyor mu? (LED yanıyor mu?)")
    print("3. Arduino IDE'de Serial Monitor çalışıyor mu?")
    print("4. Doğru kod Arduino'ya yüklendi mi?")
    
    return None

if __name__ == "__main__":
    working_port = test_arduino_connection()
    
    if working_port:
        print(f"\n🚀 Sistem hazır! Kullanılacak port: {working_port}")
        print(f"Ana sistemi başlatmak için:")
        print(f"python yolo_arduino_dart_system.py --arduino {working_port}")
    else:
        print(f"\n❌ Arduino bağlantısı kurulamadı!")
