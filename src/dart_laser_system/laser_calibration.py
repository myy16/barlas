"""
BARLAS Dart Laser System - Kalibrasyon Yardımcısı
Kamera ve lazer arasındaki offset'i kalibre etmek için
"""
import cv2
import numpy as np
import time
from dart_laser_targeting import LaserPanTiltController

class LaserCalibrationTool:
    """
    Lazer kalibrasyonu için yardımcı araç
    Kamera görüntüsündeki tık noktalarını lazer ile eşleştirir
    """
    
    def __init__(self):
        self.laser_controller = LaserPanTiltController()
        self.calibration_points = []
        self.camera_points = []
        self.laser_angles = []
        
        # Kamera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        print("[Kalibrasyon] Başlatıldı")
        print("Kullanım:")
        print("  - Mouse ile kamera görüntüsüne tıklayın")
        print("  - Lazer o noktaya yönlendirilecek")
        print("  - 'c' - Kalibrasyon noktası kaydet")
        print("  - 'r' - Kalibrasyonu sıfırla")
        print("  - 'q' - Çıkış")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Mouse tıklama eventi"""
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"[Kalibrasyon] Tık noktası: ({x}, {y})")
            
            # Lazer'i bu noktaya yönlendir
            self.laser_controller.aim_at_pixel(x, y, self.frame_width, self.frame_height)
            
            # Mevcut tık noktasını kaydet
            self.current_click = (x, y)
    
    def run_calibration(self):
        """Kalibrasyon arayüzünü çalıştırır"""
        
        cv2.namedWindow('Laser Calibration')
        cv2.setMouseCallback('Laser Calibration', self.mouse_callback)
        
        self.current_click = None
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # Kalibrasyon noktalarını göster
            for i, (cam_pt, laser_ang) in enumerate(zip(self.camera_points, self.laser_angles)):
                x, y = cam_pt
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"{i+1}", (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Mevcut tık noktası
            if self.current_click:
                x, y = self.current_click
                cv2.circle(frame, (x, y), 8, (255, 255, 0), 2)
                cv2.putText(frame, "CLICK", (x+10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Bilgi metni
            info_text = f"Kalibrasyon noktaları: {len(self.camera_points)}"
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Laser Calibration', frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('c') and self.current_click:
                # Kalibrasyon noktası kaydet
                cam_point = self.current_click
                laser_angle = (self.laser_controller.pan_position, self.laser_controller.tilt_position)
                
                self.camera_points.append(cam_point)
                self.laser_angles.append(laser_angle)
                
                print(f"[Kalibrasyon] Nokta {len(self.camera_points)} kaydedildi:")
                print(f"  Kamera: {cam_point}")
                print(f"  Laser açı: {laser_angle}")
                
                self.current_click = None
                
            elif key == ord('r'):
                # Kalibrasyonu sıfırla
                self.camera_points.clear()
                self.laser_angles.clear()
                self.current_click = None
                print("[Kalibrasyon] Sıfırlandı")
        
        # Kalibrasyon hesaplaması
        if len(self.camera_points) >= 3:
            self.calculate_calibration()
        
        self.cleanup()
    
    def calculate_calibration(self):
        """Kalibrasyon değerlerini hesaplar"""
        print("\n[Kalibrasyon] Hesaplanıyor...")
        
        # Basit linear mapping hesabı
        cam_points = np.array(self.camera_points)
        laser_angles = np.array(self.laser_angles)
        
        # Ortalama offset hesapla
        center_x = self.frame_width / 2
        center_y = self.frame_height / 2
        
        offset_x_list = []
        offset_y_list = []
        
        for (cam_x, cam_y), (pan_ang, tilt_ang) in zip(cam_points, laser_angles):
            # Kamera merkezinden fark
            cam_offset_x = cam_x - center_x
            cam_offset_y = cam_y - center_y
            
            # Servo merkezinden fark (90 derece merkez)
            servo_offset_x = pan_ang - 90
            servo_offset_y = tilt_ang - 90
            
            # Piksel başına servo açısı
            if cam_offset_x != 0:
                px_per_degree_x = servo_offset_x / cam_offset_x
                offset_x_list.append(px_per_degree_x)
            
            if cam_offset_y != 0:
                px_per_degree_y = servo_offset_y / cam_offset_y
                offset_y_list.append(px_per_degree_y)
        
        if offset_x_list and offset_y_list:
            avg_offset_x = np.mean(offset_x_list)
            avg_offset_y = np.mean(offset_y_list)
            
            print(f"Kalibrasyon sonuçları:")
            print(f"  X offset: {avg_offset_x:.4f} derece/piksel")
            print(f"  Y offset: {avg_offset_y:.4f} derece/piksel")
            
            # Kalibrasyonu kaydet
            with open("laser_calibration.txt", "w") as f:
                f.write(f"x_offset={avg_offset_x}\n")
                f.write(f"y_offset={avg_offset_y}\n")
            
            print("Kalibrasyon laser_calibration.txt dosyasına kaydedildi")
        
        else:
            print("Kalibrasyon hesaplanamadı - yetersiz veri")
    
    def cleanup(self):
        """Temizlik"""
        self.cap.release()
        cv2.destroyAllWindows()
        self.laser_controller.cleanup()


if __name__ == "__main__":
    print("=== BARLAS Lazer Kalibrasyon Aracı ===")
    
    try:
        calibrator = LaserCalibrationTool()
        calibrator.run_calibration()
        
    except Exception as e:
        print(f"Kalibrasyon hatası: {e}")
        import traceback
        traceback.print_exc()
