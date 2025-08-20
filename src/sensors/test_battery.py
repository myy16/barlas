"""
Batarya sensörü test modülü
"""
import time
from battery_sensor import BatterySensor

def test_battery_sensor():
    """Batarya sensörünü test et"""
    print("🔋 Batarya sensör testi başlatılıyor...")
    
    # Sensörü başlat
    sensor = BatterySensor()
    
    if not sensor.initialize():
        print("❌ Sensör başlatılamadı!")
        return
        
    try:
        # Sürekli okuma başlat
        sensor.start_reading()
        
        # Birkaç ölçüm al
        for i in range(10):
            reading = sensor.get_last_reading()
            age = sensor.get_reading_age()
            battery_percent = sensor.get_battery_percentage()
            
            if reading:
                print(f"\nÖlçüm #{i+1}")
                print(f"Voltaj: {reading['bus_voltage']:.2f}V")
                print(f"Akım: {reading['current']:.0f}mA")
                print(f"Güç: {reading['power']:.1f}mW")
                if battery_percent is not None:
                    print(f"Batarya: {battery_percent:.0f}%")
                print(f"Ölçüm yaşı: {age:.1f}s")
            else:
                print("\n⚠️ Ölçüm hatası!")
                
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruluyor...")
    finally:
        sensor.cleanup()
        print("✅ Test tamamlandı")

def monitor_battery():
    """Sürekli batarya izleme"""
    print("📊 Batarya izleme başlatılıyor...")
    
    sensor = BatterySensor()
    
    if not sensor.initialize():
        print("❌ Sensör başlatılamadı!")
        return
        
    try:
        sensor.start_reading()
        
        while True:
            reading = sensor.get_last_reading()
            battery_percent = sensor.get_battery_percentage()
            
            if reading and battery_percent is not None:
                # Batarya durumuna göre renklendirme
                if battery_percent > 50:
                    color = '\033[92m'  # Yeşil
                elif battery_percent > 20:
                    color = '\033[93m'  # Sarı
                else:
                    color = '\033[91m'  # Kırmızı
                    
                reset = '\033[0m'
                print(f"\rBatarya: {color}{battery_percent:.0f}%{reset} | " + 
                      f"Voltaj: {reading['bus_voltage']:.2f}V | " +
                      f"Akım: {reading['current']:.0f}mA", end='')
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n🛑 İzleme durduruluyor...")
    finally:
        sensor.cleanup()
        print("✅ İzleme tamamlandı")

if __name__ == "__main__":
    print("INA219 Test Menüsü")
    print("1. Temel Test")
    print("2. Sürekli İzleme")
    
    choice = input("Test seçin (1/2): ")
    
    if choice == "1":
        test_battery_sensor()
    elif choice == "2":
        monitor_battery()
    else:
        print("❌ Geçersiz seçim")
