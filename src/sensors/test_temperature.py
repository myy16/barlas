#!/usr/bin/env python3
"""
BARLAS Temperature Sensor Test
BME280 sıcaklık/nem/basınç ölçüm testi
Teknofest İnsansız Kara Aracı Yarışması - Su sızıntısı ve şasi sıcaklık takibi
"""

import time
import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

from temperature_sensor import TemperatureSensor

def test_temperature_sensor():
    """Temperature sensor test fonksiyonu"""
    print("🌡️ BARLAS Temperature Sensor Test Başlıyor...")
    
    try:
        # Temperature sensor initialize
        temp_sensor = TemperatureSensor()
        
        print("\n📊 Çevre Koşulları Testi:")
        
        for i in range(15):  # 15 ölçüm
            temperature = temp_sensor.get_temperature()
            humidity = temp_sensor.get_humidity()
            pressure = temp_sensor.get_pressure()
            
            # Durum analizi
            temp_status = "🔴 SICAK" if temperature > 50 else \
                         "🟡 ILIK" if temperature > 30 else \
                         "🟢 NORMAL"
            
            hum_status = "💧 NEMLI" if humidity > 80 else \
                        "🟡 ORTA" if humidity > 50 else \
                        "☀️ KURU"
            
            # Su sızıntısı riski
            water_risk = "🚨 SU RİSKİ!" if humidity > 90 and temperature < 20 else "✅ GÜVENLİ"
            
            print(f"Test {i+1:2d}: "
                  f"Sıcaklık: {temperature:5.1f}°C {temp_status} | "
                  f"Nem: %{humidity:4.1f} {hum_status} | "
                  f"Basınç: {pressure:7.1f}Pa | {water_risk}")
            
            time.sleep(2)
        
        print("\n🎯 Su Sızıntısı Alarm Testi:")
        print("Sensörü nemli ortama yaklaştırın...")
        
        for i in range(10):
            temperature = temp_sensor.get_temperature()
            humidity = temp_sensor.get_humidity()
            
            # Kritik nem seviyesi
            if humidity > 95:
                print(f"🚨 ALARM! Nem seviyesi: %{humidity:.1f} - Su sızıntısı riski!")
            elif humidity > 85:
                print(f"⚠️ DİKKAT! Nem seviyesi: %{humidity:.1f} - Kontrol edin!")
            else:
                print(f"✅ Normal - Nem: %{humidity:.1f}, Sıcaklık: {temperature:.1f}°C")
            
            time.sleep(1.5)
        
        print("\n🌡️ Sıcaklık Alarm Testi:")
        print("Sistem sıcaklığı monitörü...")
        
        for i in range(10):
            temperature = temp_sensor.get_temperature()
            
            if temperature > 60:
                print(f"🔥 AŞIRI SICAK! {temperature:.1f}°C - Sistem kapatılmalı!")
            elif temperature > 45:
                print(f"🟡 SICAK! {temperature:.1f}°C - Soğutma gerekli!")
            else:
                print(f"❄️ Normal sıcaklık: {temperature:.1f}°C")
            
            time.sleep(1)
            
    except Exception as e:
        print(f"❌ Temperature sensor hatası: {e}")
        return False
    
    print("✅ Temperature sensor testi tamamlandı!")
    return True

if __name__ == "__main__":
    test_temperature_sensor()
