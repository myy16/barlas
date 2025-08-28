#!/usr/bin/env python3
"""
BARLAS IMU Sensor Test
MPU9250 ivme/jiroskop/manyetometre testi
Teknofest İnsansız Kara Aracı Yarışması - Eğim ve Denge Kontrolü
"""

import time
import math
import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

from imu_sensor import IMUSensor

def test_imu_sensor():
    """IMU sensor test fonksiyonu"""
    print("🧭 BARLAS IMU Sensor Test Başlıyor...")
    
    try:
        # IMU sensor initialize
        imu = IMUSensor()
        
        print("\n📊 IMU Kalibrasyonu ve Test:")
        print("⚠️  Aracı düz zemine yerleştirin...")
        time.sleep(3)
        
        # Kalibrasyon
        imu.calibrate()
        print("✅ Kalibrasyon tamamlandı!")
        
        print("\n🔄 IMU Verileri (10 saniye):")
        
        for i in range(50):  # 10 saniye x 5Hz
            # Açısal veriler
            roll, pitch, yaw = imu.get_orientation()
            
            # İvme verileri
            accel_x, accel_y, accel_z = imu.get_acceleration()
            
            # Jiroskop verileri  
            gyro_x, gyro_y, gyro_z = imu.get_gyroscope()
            
            # Eğim kontrolü
            tilt_angle = math.sqrt(roll**2 + pitch**2)
            
            print(f"\rTest {i+1}/50 | "
                  f"Roll: {roll:6.1f}° | "
                  f"Pitch: {pitch:6.1f}° | "
                  f"Yaw: {yaw:6.1f}° | "
                  f"Eğim: {tilt_angle:5.1f}° | "
                  f"{'🔴 EĞİM!' if tilt_angle > 25 else '🟢 DENGEDE'}", 
                  end="", flush=True)
            
            time.sleep(0.2)
        
        print("\n\n🎯 Eğim Testi:")
        print("Aracı farklı açılarda eğin...")
        
        for i in range(25):
            roll, pitch, yaw = imu.get_orientation()
            tilt_angle = math.sqrt(roll**2 + pitch**2)
            
            status = "🔴 TEHLİKELİ" if tilt_angle > 45 else \
                     "🟡 DİKKAT" if tilt_angle > 25 else \
                     "🟢 GÜVENLİ"
            
            print(f"Eğim Açısı: {tilt_angle:5.1f}° | {status}")
            time.sleep(0.5)
            
    except Exception as e:
        print(f"\n❌ IMU sensor hatası: {e}")
        return False
    
    print("\n✅ IMU sensor testi tamamlandı!")
    return True

if __name__ == "__main__":
    test_imu_sensor()
