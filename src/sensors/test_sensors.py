"""
Sensör test modülü
"""
import time
from ultrasonic_manager import UltrasonicManager
from obstacle_avoidance import ObstacleAvoidance

def test_ultrasonic_sensors():
    """Ultrasonik sensörleri test et"""
    print("🔍 Ultrasonik sensör testi başlatılıyor...")
    
    # Sensör yöneticisini başlat
    manager = UltrasonicManager()
    manager.start_measurements()
    
    try:
        while True:
            # Tüm mesafeleri al
            distances = manager.get_all_distances()
            
            # Sonuçları yazdır
            print("\n--- Sensör Mesafeleri ---")
            print("Ön Sensörler:")
            for i in range(1, 5):
                print(f"  Sensör {i}: {distances[i]:.1f} cm")
            
            print("\nArka Sensörler:")
            for i in range(5, 9):
                print(f"  Sensör {i}: {distances[i]:.1f} cm")
            
            # Engel durumunu kontrol et
            obstacles = manager.get_obstacle_status()
            print("\nEngel Durumu:")
            for sid, has_obstacle in obstacles.items():
                if has_obstacle:
                    print(f"⚠️ Sensör {sid}: ENGEL VAR!")
            
            time.sleep(0.5)  # 2Hz güncelleme
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruluyor...")
    finally:
        manager.cleanup()
        print("✅ Test tamamlandı")

def test_obstacle_avoidance():
    """Engelden kaçış algoritmasını test et"""
    print("🤖 Engelden kaçış testi başlatılıyor...")
    
    # Engelden kaçış sistemini başlat
    avoidance = ObstacleAvoidance()
    
    try:
        while True:
            # Hareket komutlarını hesapla
            linear_speed, angular_speed = avoidance.calculate_avoidance_vector()
            
            # Sonuçları yazdır
            print("\n--- Hareket Komutları ---")
            print(f"Doğrusal Hız: {linear_speed:.2f} m/s")
            print(f"Açısal Hız: {angular_speed:.2f} rad/s")
            
            # Yol durumunu kontrol et
            if avoidance.is_path_clear():
                print("✅ Yol açık")
            else:
                print("⚠️ Engel tespit edildi!")
            
            time.sleep(0.2)  # 5Hz güncelleme
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruluyor...")
    finally:
        avoidance.cleanup()
        print("✅ Test tamamlandı")

if __name__ == "__main__":
    # Hangi testi çalıştırmak istediğinizi seçin
    print("1. Ultrasonik Sensör Testi")
    print("2. Engelden Kaçış Testi")
    choice = input("Test seçin (1/2): ")
    
    if choice == "1":
        test_ultrasonic_sensors()
    elif choice == "2":
        test_obstacle_avoidance()
    else:
        print("❌ Geçersiz seçim")
