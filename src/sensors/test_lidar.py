"""
LIDAR sensör test modülü
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from lidar_manager import LidarManager

def test_lidar_connection():
    """LIDAR bağlantısını test et"""
    print("🔍 LIDAR bağlantı testi başlatılıyor...")
    
    # Windows için COM portu, Linux için /dev/ttyUSB0
    port = "COM3" if os.name == 'nt' else "/dev/ttyUSB0"
    
    # LIDAR yöneticisini başlat
    lidar = LidarManager(port=port)
    
    if not lidar.connect():
        print("❌ LIDAR bağlantısı başarısız!")
        return
        
    try:
        # Taramayı başlat
        lidar.start_scanning()
        
        # Birkaç tarama al ve göster
        for i in range(5):
            scan = lidar.get_latest_scan()
            if scan:
                print(f"\nTarama #{i+1}:")
                print(f"Nokta sayısı: {len(scan['angles'])}")
                print(f"Min mesafe: {min(scan['distances']):.2f}m")
                print(f"Max mesafe: {max(scan['distances']):.2f}m")
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruluyor...")
    finally:
        lidar.cleanup()
        print("✅ Test tamamlandı")

def visualize_lidar_scan():
    """LIDAR taramasını görselleştir"""
    print("📊 LIDAR görselleştirme başlatılıyor...")
    
    port = "COM3" if os.name == 'nt' else "/dev/ttyUSB0"
    lidar = LidarManager(port=port)
    
    if not lidar.connect():
        print("❌ LIDAR bağlantısı başarısız!")
        return
        
    try:
        # Taramayı başlat
        lidar.start_scanning()
        
        # Matplotlib figure hazırla
        plt.ion()  # Interaktif mod
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='polar')
        
        while True:
            # Tarama verisini al
            scan = lidar.get_latest_scan()
            if scan:
                # Polar plot temizle
                ax.clear()
                
                # Radyal veriyi çiz
                angles_rad = np.deg2rad(scan['angles'])
                ax.scatter(angles_rad, scan['distances'], 
                         c=scan['intensities'], cmap='viridis', 
                         s=5, alpha=0.5)
                
                # Grafik ayarları
                ax.set_title('YDLIDAR X4 Taraması')
                ax.set_theta_zero_location('N')  # 0 derece yukarıda
                ax.set_theta_direction(-1)       # Saat yönünde
                ax.grid(True)
                
                # Ekranı güncelle
                plt.draw()
                plt.pause(0.1)
                
    except KeyboardInterrupt:
        print("\n🛑 Görselleştirme durduruluyor...")
    finally:
        lidar.cleanup()
        plt.ioff()
        plt.close()
        print("✅ Görselleştirme tamamlandı")

def test_obstacle_detection():
    """Engel tespitini test et"""
    print("🎯 Engel tespit testi başlatılıyor...")
    
    port = "COM3" if os.name == 'nt' else "/dev/ttyUSB0"
    lidar = LidarManager(port=port)
    
    if not lidar.connect():
        print("❌ LIDAR bağlantısı başarısız!")
        return
        
    try:
        # Taramayı başlat
        lidar.start_scanning()
        
        while True:
            # Yakın engelleri tespit et (0.1m - 1.0m arası)
            obstacles = lidar.get_obstacles_in_range(0.1, 1.0)
            
            if obstacles:
                print("\n⚠️ Engeller tespit edildi!")
                for angle, distance in obstacles:
                    print(f"  {angle:.1f}° yönünde {distance:.2f}m mesafede")
            else:
                print("\n✅ Yakın çevrede engel yok")
                
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruluyor...")
    finally:
        lidar.cleanup()
        print("✅ Test tamamlandı")

if __name__ == "__main__":
    print("YDLIDAR X4 Test Menüsü")
    print("1. Bağlantı Testi")
    print("2. Görselleştirme")
    print("3. Engel Tespiti")
    
    choice = input("Test seçin (1/2/3): ")
    
    if choice == "1":
        test_lidar_connection()
    elif choice == "2":
        visualize_lidar_scan()
    elif choice == "3":
        test_obstacle_detection()
    else:
        print("❌ Geçersiz seçim")
