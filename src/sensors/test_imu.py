"""
IMU sensörü test modülü
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from imu_sensor import IMUSensor

def test_imu_sensor():
    """IMU sensörünü test et"""
    print("🧭 IMU sensör testi başlatılıyor...")
    
    # Sensörü başlat
    imu = IMUSensor()
    
    if not imu.initialize():
        print("❌ Sensör başlatılamadı!")
        return
        
    try:
        # Sürekli okuma başlat
        imu.start_reading()
        
        # Birkaç ölçüm al
        for i in range(10):
            reading = imu.get_last_reading()
            orientation = imu.get_orientation()
            age = imu.get_reading_age()
            
            if reading and orientation:
                print(f"\nÖlçüm #{i+1}")
                print("İvmeölçer (g):")
                print(f"  X: {reading['accelerometer'][0]:.2f}")
                print(f"  Y: {reading['accelerometer'][1]:.2f}")
                print(f"  Z: {reading['accelerometer'][2]:.2f}")
                
                print("\nJiroskop (derece/s):")
                print(f"  X: {reading['gyroscope'][0]:.1f}")
                print(f"  Y: {reading['gyroscope'][1]:.1f}")
                print(f"  Z: {reading['gyroscope'][2]:.1f}")
                
                print("\nManyetometre (uT):")
                print(f"  X: {reading['magnetometer'][0]:.1f}")
                print(f"  Y: {reading['magnetometer'][1]:.1f}")
                print(f"  Z: {reading['magnetometer'][2]:.1f}")
                
                print("\nYönelim (derece):")
                print(f"  Roll: {orientation['roll']:.1f}")
                print(f"  Pitch: {orientation['pitch']:.1f}")
                print(f"  Yaw: {orientation['yaw']:.1f}")
                
                print(f"\nÖlçüm yaşı: {age:.3f}s")
            else:
                print("\n⚠️ Ölçüm hatası!")
                
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruluyor...")
    finally:
        imu.cleanup()
        print("✅ Test tamamlandı")

def visualize_orientation():
    """IMU yönelimini gerçek zamanlı görselleştir"""
    print("📊 IMU görselleştirme başlatılıyor...")
    
    imu = IMUSensor()
    
    if not imu.initialize():
        print("❌ Sensör başlatılamadı!")
        return
        
    try:
        # Sürekli okuma başlat
        imu.start_reading()
        
        # Matplotlib figure hazırla
        plt.ion()
        fig = plt.figure(figsize=(12, 4))
        
        # 3 grafik: Roll, Pitch, Yaw
        ax1 = fig.add_subplot(131)
        ax2 = fig.add_subplot(132)
        ax3 = fig.add_subplot(133)
        
        # Veri tamponları
        buffer_size = 100
        time_data = np.zeros(buffer_size)
        roll_data = np.zeros(buffer_size)
        pitch_data = np.zeros(buffer_size)
        yaw_data = np.zeros(buffer_size)
        
        t_start = time.time()
        
        while True:
            # Yönelim verisini al
            orientation = imu.get_orientation()
            
            if orientation:
                # Veri tamponlarını kaydır
                time_data = np.roll(time_data, -1)
                roll_data = np.roll(roll_data, -1)
                pitch_data = np.roll(pitch_data, -1)
                yaw_data = np.roll(yaw_data, -1)
                
                # Yeni veriyi ekle
                t = time.time() - t_start
                time_data[-1] = t
                roll_data[-1] = orientation['roll']
                pitch_data[-1] = orientation['pitch']
                yaw_data[-1] = orientation['yaw']
                
                # Grafikleri güncelle
                ax1.clear()
                ax2.clear()
                ax3.clear()
                
                ax1.plot(time_data, roll_data, 'r-')
                ax1.set_title('Roll')
                ax1.set_ylim(-180, 180)
                ax1.grid(True)
                
                ax2.plot(time_data, pitch_data, 'g-')
                ax2.set_title('Pitch')
                ax2.set_ylim(-90, 90)
                ax2.grid(True)
                
                ax3.plot(time_data, yaw_data, 'b-')
                ax3.set_title('Yaw')
                ax3.set_ylim(0, 360)
                ax3.grid(True)
                
                plt.tight_layout()
                plt.draw()
                plt.pause(0.01)
            
    except KeyboardInterrupt:
        print("\n🛑 Görselleştirme durduruluyor...")
    finally:
        imu.cleanup()
        plt.ioff()
        plt.close()
        print("✅ Görselleştirme tamamlandı")

if __name__ == "__main__":
    print("MPU9250 Test Menüsü")
    print("1. Temel Test")
    print("2. Yönelim Görselleştirme")
    
    choice = input("Test seçin (1/2): ")
    
    if choice == "1":
        test_imu_sensor()
    elif choice == "2":
        visualize_orientation()
    else:
        print("❌ Geçersiz seçim")
