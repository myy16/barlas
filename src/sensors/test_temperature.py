"""
Sıcaklık sensörü test modülü
"""
import time
from temperature_sensor import TemperatureSensor

def test_temperature_sensor():
    """Sıcaklık sensörünü test et"""
    print("🌡️ Sıcaklık sensör testi başlatılıyor...")
    
    # Sensörü başlat
    sensor = TemperatureSensor()
    
    if not sensor.initialize():
        print("❌ Sensör başlatılamadı!")
        return
        
    try:
        # Sürekli okuma başlat
        sensor.start_reading()
        
        # Birkaç ölçüm al
        for i in range(10):
            temp = sensor.get_last_temperature()
            age = sensor.get_reading_age()
            
            if temp is not None:
                print(f"\nÖlçüm #{i+1}")
                print(f"Sıcaklık: {temp:.1f}°C")
                print(f"Ölçüm yaşı: {age:.1f} saniye")
            else:
                print("\n⚠️ Ölçüm hatası!")
                
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruluyor...")
    finally:
        sensor.cleanup()
        print("✅ Test tamamlandı")

def monitor_temperature():
    """Sürekli sıcaklık izleme"""
    print("📊 Sıcaklık izleme başlatılıyor...")
    
    sensor = TemperatureSensor()
    
    if not sensor.initialize():
        print("❌ Sensör başlatılamadı!")
        return
        
    try:
        sensor.start_reading()
        
        while True:
            temp = sensor.get_last_temperature()
            if temp is not None:
                # ANSI renk kodları ile sıcaklığa göre renklendirme
                if temp > 30:
                    color = '\033[91m'  # Kırmızı
                elif temp > 25:
                    color = '\033[93m'  # Sarı
                else:
                    color = '\033[92m'  # Yeşil
                    
                reset = '\033[0m'
                print(f"\rSıcaklık: {color}{temp:.1f}°C{reset}", end='')
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n🛑 İzleme durduruluyor...")
    finally:
        sensor.cleanup()
        print("✅ İzleme tamamlandı")

if __name__ == "__main__":
    print("DS18B20 Test Menüsü")
    print("1. Temel Test")
    print("2. Sürekli İzleme")
    
    choice = input("Test seçin (1/2): ")
    
    if choice == "1":
        test_temperature_sensor()
    elif choice == "2":
        monitor_temperature()
    else:
        print("❌ Geçersiz seçim")
