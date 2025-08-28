#!/usr/bin/env python3
"""
BARLAS LiDAR Test
Engel algılama ve haritalama testi
Teknofest İnsansız Kara Aracı Yarışması
"""

import time
import numpy as np
import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

from lidar_manager import LidarManager

def test_lidar_sensor():
    """LiDAR sensor test fonksiyonu"""
    print("📡 BARLAS LiDAR Test Başlıyor...")
    
    try:
        # LiDAR initialize
        lidar = LidarManager()
        
        print("\n🔄 LiDAR Tarama Testi:")
        
        for i in range(20):  # 20 saniye test
            # Tam tarama al
            scan_data = lidar.get_scan()
            
            if scan_data is None:
                print(f"Test {i+1}: ❌ Veri alınamadı")
                continue
            
            # Engel analizi
            front_distances = []
            left_distances = []
            right_distances = []
            
            angles = scan_data['angles']
            ranges = scan_data['ranges']
            
            for j, angle in enumerate(angles):
                distance = ranges[j]
                
                # Ön bölge (-45° to +45°)
                if -45 <= np.degrees(angle) <= 45:
                    front_distances.append(distance)
                # Sol bölge (45° to 135°)  
                elif 45 <= np.degrees(angle) <= 135:
                    left_distances.append(distance)
                # Sağ bölge (-135° to -45°)
                elif -135 <= np.degrees(angle) <= -45:
                    right_distances.append(distance)
            
            # En yakın engeller
            min_front = min(front_distances) if front_distances else float('inf')
            min_left = min(left_distances) if left_distances else float('inf')
            min_right = min(right_distances) if right_distances else float('inf')
            
            # Durum analizi
            front_status = "🔴 ENGELLİ" if min_front < 1.5 else "🟢 AÇIK"
            left_status = "🔴 ENGELLİ" if min_left < 1.0 else "🟢 AÇIK"
            right_status = "🔴 ENGELLİ" if min_right < 1.0 else "🟢 AÇIK"
            
            print(f"Test {i+1:2d} | "
                  f"Ön: {min_front:5.1f}m {front_status} | "
                  f"Sol: {min_left:5.1f}m {left_status} | "
                  f"Sağ: {min_right:5.1f}m {right_status}")
            
            time.sleep(1)
        
        print("\n🎯 Parkur Engel Testi:")
        print("Aracı engellerin yanına götürün...")
        
        obstacle_detected = False
        for i in range(30):
            scan_data = lidar.get_scan()
            if scan_data is None:
                continue
                
            # Kritik mesafe kontrolü
            critical_distances = [r for r in scan_data['ranges'] if 0.1 < r < 0.5]
            
            if len(critical_distances) > 10:  # Çok yakın engel
                if not obstacle_detected:
                    print("🚨 KRİTİK MESAFE! Engel çok yakın!")
                    obstacle_detected = True
            else:
                if obstacle_detected:
                    print("✅ Güvenli mesafe")
                    obstacle_detected = False
            
            time.sleep(0.5)
            
    except Exception as e:
        print(f"❌ LiDAR hatası: {e}")
        return False
    
    print("✅ LiDAR testi tamamlandı!")
    return True

if __name__ == "__main__":
    test_lidar_sensor()
