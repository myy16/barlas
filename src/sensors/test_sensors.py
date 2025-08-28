#!/usr/bin/env python3
"""
BARLAS Comprehensive Sensor Test Suite
Tüm sensörlerin entegre testi
Teknofest İnsansız Kara Aracı Yarışması
"""

import time
import threading
import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

from battery_sensor import BatterySensor
from imu_sensor import IMUSensor
from lidar_manager import LidarManager
from temperature_sensor import TemperatureSensor
from ultrasonic_manager import UltrasonicManager

class BARLASSensorTestSuite:
    def __init__(self):
        """Tüm sensörleri initialize et"""
        print("🚗 BARLAS Sensor Test Suite Başlıyor...")
        
        self.sensors = {}
        self.test_running = False
        
        try:
            print("🔋 Battery sensor initialize...")
            self.sensors['battery'] = BatterySensor()
            
            print("🧭 IMU sensor initialize...")
            self.sensors['imu'] = IMUSensor()
            
            print("📡 LiDAR initialize...")
            self.sensors['lidar'] = LidarManager()
            
            print("🌡️ Temperature sensor initialize...")
            self.sensors['temperature'] = TemperatureSensor()
            
            print("📏 Ultrasonic sensors initialize...")
            self.sensors['ultrasonic'] = UltrasonicManager()
            
            print("✅ Tüm sensörler hazır!")
            
        except Exception as e:
            print(f"❌ Sensor initialization hatası: {e}")
    
    def test_all_sensors_continuous(self, duration=30):
        """Tüm sensörleri sürekli test et"""
        print(f"\n🔄 {duration} saniye sürekli sensor testi başlıyor...")
        
        self.test_running = True
        start_time = time.time()
        
        while self.test_running and (time.time() - start_time) < duration:
            try:
                # Battery status
                voltage = self.sensors['battery'].get_voltage()
                battery_pct = self.sensors['battery'].get_battery_percentage()
                
                # IMU orientation
                roll, pitch, yaw = self.sensors['imu'].get_orientation()
                
                # LiDAR obstacle detection
                scan_data = self.sensors['lidar'].get_scan()
                min_distance = min(scan_data['ranges']) if scan_data else float('inf')
                
                # Temperature/humidity
                temperature = self.sensors['temperature'].get_temperature()
                humidity = self.sensors['temperature'].get_humidity()
                
                # Ultrasonic distances
                ultrasonic_data = self.sensors['ultrasonic'].get_all_distances()
                min_ultrasonic = min(ultrasonic_data.values()) if ultrasonic_data else float('inf')
                
                # Status display
                elapsed = int(time.time() - start_time)
                print(f"\r⏱️ {elapsed:2d}s | "
                      f"🔋{battery_pct:4.1f}% | "
                      f"🧭{roll:5.1f}°/{pitch:5.1f}° | "
                      f"📡{min_distance:4.1f}m | "
                      f"📏{min_ultrasonic:4.1f}m | "
                      f"🌡️{temperature:4.1f}°C", end="", flush=True)
                
                time.sleep(0.5)
                
            except KeyboardInterrupt:
                print("\n⏹️ Test manuel olarak durduruldu")
                break
            except Exception as e:
                print(f"\n❌ Test hatası: {e}")
                time.sleep(1)
        
        self.test_running = False
        print(f"\n✅ Sürekli test tamamlandı! ({elapsed} saniye)")
    
    def test_competition_scenario(self):
        """Yarışma senaryosu testi"""
        print("\n🏁 TEKNOFEST Yarışma Senaryosu Testi")
        
        scenarios = [
            "🚗 Düz yol sürüşü",
            "🔄 Engel kaçınma",
            "⛰️ Eğim tırmanma", 
            "💧 Su geçişi",
            "🎯 Hedef vurma pozisyonu"
        ]
        
        for i, scenario in enumerate(scenarios):
            print(f"\n📋 Senaryo {i+1}: {scenario}")
            
            for j in range(10):  # Her senaryo 10 saniye
                try:
                    # Critical sensor readings
                    battery_pct = self.sensors['battery'].get_battery_percentage()
                    roll, pitch, yaw = self.sensors['imu'].get_orientation()
                    
                    # Scenario-specific checks
                    if i == 1:  # Engel kaçınma
                        scan_data = self.sensors['lidar'].get_scan()
                        obstacle_distance = min(scan_data['ranges']) if scan_data else float('inf')
                        status = "🔴 ENGEL!" if obstacle_distance < 1.5 else "🟢 YOL AÇIK"
                        print(f"  {j+1:2d}/10: Engel mesafesi: {obstacle_distance:4.1f}m - {status}")
                        
                    elif i == 2:  # Eğim tırmanma
                        tilt_angle = (roll**2 + pitch**2)**0.5
                        status = "⚠️ DİKKAT!" if tilt_angle > 25 else "✅ STABIL"
                        print(f"  {j+1:2d}/10: Eğim açısı: {tilt_angle:4.1f}° - {status}")
                        
                    elif i == 3:  # Su geçişi
                        humidity = self.sensors['temperature'].get_humidity()
                        status = "💧 SU TESPİTİ!" if humidity > 85 else "🏜️ KURU"
                        print(f"  {j+1:2d}/10: Nem oranı: %{humidity:4.1f} - {status}")
                        
                    else:  # Genel durum
                        print(f"  {j+1:2d}/10: Batarya: %{battery_pct:4.1f} - Açı: {roll:4.1f}°/{pitch:4.1f}°")
                    
                    time.sleep(1)
                    
                except Exception as e:
                    print(f"  ❌ Senaryo {i+1} hatası: {e}")
        
        print("\n🏆 Yarışma senaryosu testi tamamlandı!")
    
    def emergency_shutdown_test(self):
        """Acil durum sensör testi"""
        print("\n🚨 Acil Durum Sensör Testi")
        
        emergency_conditions = []
        
        for i in range(20):
            try:
                # Battery critical
                battery_pct = self.sensors['battery'].get_battery_percentage()
                if battery_pct < 15:
                    emergency_conditions.append("🔋 KRİTİK BATARYA SEVIYESI!")
                
                # Extreme tilt
                roll, pitch, yaw = self.sensors['imu'].get_orientation()
                tilt_angle = (roll**2 + pitch**2)**0.5
                if tilt_angle > 45:
                    emergency_conditions.append("⚠️ ARAÇ DEVRILME RİSKİ!")
                
                # Obstacle collision risk
                scan_data = self.sensors['lidar'].get_scan()
                if scan_data:
                    min_distance = min(scan_data['ranges'])
                    if min_distance < 0.3:
                        emergency_conditions.append("🚧 ÇARPMA RİSKİ!")
                
                # Overheating
                temperature = self.sensors['temperature'].get_temperature()
                if temperature > 55:
                    emergency_conditions.append("🔥 AŞIRI SICAKLIK!")
                
                # Display status
                if emergency_conditions:
                    print(f"Test {i+1:2d}: 🚨 ACİL DURUM ALGILANDI!")
                    for condition in emergency_conditions:
                        print(f"    {condition}")
                    emergency_conditions.clear()
                else:
                    print(f"Test {i+1:2d}: ✅ Normal durum - Güvenli")
                
                time.sleep(1.5)
                
            except Exception as e:
                print(f"❌ Acil durum test hatası: {e}")
        
        print("✅ Acil durum testi tamamlandı!")

def main():
    """Ana test fonksiyonu"""
    test_suite = BARLASSensorTestSuite()
    
    while True:
        print("\n" + "="*60)
        print("🚗 BARLAS SENSOR TEST SUITE")
        print("="*60)
        print("1. Sürekli Sensor Testi (30s)")
        print("2. Yarışma Senaryosu Testi")
        print("3. Acil Durum Testi")
        print("4. Çıkış")
        
        choice = input("\nSeçiminizi yapın (1-4): ").strip()
        
        if choice == '1':
            test_suite.test_all_sensors_continuous()
        elif choice == '2':
            test_suite.test_competition_scenario()
        elif choice == '3':
            test_suite.emergency_shutdown_test()
        elif choice == '4':
            print("👋 Test suite kapatılıyor...")
            break
        else:
            print("❌ Geçersiz seçim!")

if __name__ == "__main__":
    main()
