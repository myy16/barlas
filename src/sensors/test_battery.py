#!/usr/bin/env python3
"""
BARLAS Battery Sensor Test
INA219 voltaj/akım ölçüm testi
Teknofest İnsansız Kara Aracı Yarışması
"""

import time
import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

from battery_sensor import BatterySensor

def test_battery_sensor():
    """Battery sensor test fonksiyonu"""
    print("🔋 BARLAS Battery Sensor Test Başlıyor...")
    
    try:
        # Battery sensor initialize
        battery = BatterySensor()
        
        print("\n📊 Batarya Durumu Testi:")
        
        for i in range(10):
            voltage = battery.get_voltage()
            current = battery.get_current()
            power = battery.get_power()
            percentage = battery.get_battery_percentage()
            
            print(f"Test {i+1}:")
            print(f"  Voltaj: {voltage:.2f}V")
            print(f"  Akım: {current:.3f}A")
            print(f"  Güç: {power:.2f}W")
            print(f"  Batarya: %{percentage:.1f}")
            print(f"  Durum: {'🔴 DÜŞÜK' if percentage < 20 else '🟡 ORTA' if percentage < 50 else '🟢 İYİ'}")
            print("-" * 30)
            
            time.sleep(2)
            
    except Exception as e:
        print(f"❌ Battery sensor hatası: {e}")
        return False
    
    print("✅ Battery sensor testi tamamlandı!")
    return True

if __name__ == "__main__":
    test_battery_sensor()
