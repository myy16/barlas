"""
BARLAS Interactive Test Suite
Arduino Simülatörü ile tam sistem testi
"""
import cv2
import numpy as np
import time
import threading
import os
import sys

# Modülleri import et
from dart_detector import DartDetector
from arduino_simulator import ArduinoSimulator

class BarlasInteractiveTest:
    """BARLAS entegre test sistemi"""
    
    def __init__(self):
        """Test sistemi başlat"""
        print("🎯 BARLAS Interactive Test Suite")
        print("=" * 50)
        
        # Alt sistemleri başlat
        print("🔧 Sistemler başlatılıyor...")
        
        # Dart detector - optimize edilmiş
        self.dart_detector = DartDetector(confidence_threshold=0.4)
        self.dart_detector.set_hough_params(
            param1=60,           # Optimize edilmiş
            param2=20,           # Daha hassas
            minDist_ratio=0.25,  
            minRadius_ratio=0.08, # Daha küçük dart'lara izin
            blur_kernel=5,       # Daha az blur
            blur_sigma=1.2
        )
        
        # Arduino simulator
        self.arduino = ArduinoSimulator()
        
        # Test durumu
        self.is_running = False
        self.target_locked = False
        self.lock_start_time = None
        self.lock_duration = 2.0  # Kilitlenme süresi
        
        print("✅ Tüm sistemler hazır!")
    
    def run_full_system_test(self):
        """Tam sistem entegrasyonu testi"""
        print("🚀 Tam sistem testi başlatılıyor...")
        
        # Kamera başlat
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Kamera açılamadı!")
            return
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("✅ Kamera hazır!")
        
        # Arduino'yu merkeze getir
        self.arduino.center_position()
        
        print("📋 Kontroller:")
        print("  ESC: Çıkış")
        print("  SPACE: Manuel lazer ateşi")
        print("  Arrow Keys: Manuel servo hareketi")
        print("  H: Hough parametrelerini göster")
        print("  T: Test hareket deseni")
        print("=" * 50)
        
        frame_count = 0
        last_target_center = None
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("❌ Kamera görüntüsü alınamadı!")
                    break
                
                frame_count += 1
                start_time = time.time()
                
                # Dart detection
                detections = self.dart_detector.detect_darts(frame)
                
                # En iyi dart'ı seç
                best_dart = None
                if detections:
                    # Güvenilir dart'ları filtrele
                    reliable_darts = [d for d in detections if d['confidence'] >= 0.5]
                    
                    if reliable_darts:
                        # En büyük area'ya sahip olanı seç
                        best_dart = max(reliable_darts, key=lambda d: d['area'])
                        
                        # Hough circle refinement uygula
                        best_dart = self.dart_detector.get_best_dart([best_dart], frame)
                
                process_time = (time.time() - start_time) * 1000
                
                # Hedef takibi ve lazer kontrolü
                self.handle_targeting(best_dart, last_target_center)
                
                # Görselleştirme
                display_frame = self.draw_system_status(frame, detections, best_dart, process_time, frame_count)
                
                # Display
                cv2.imshow('BARLAS Full System Test', display_frame)
                
                # Klavye kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
                elif key == ord(' '):  # SPACE - Manuel ateş
                    print("🔴 Manuel lazer ateşi!")
                    self.arduino.fire_laser(1.0)
                elif key == 82:  # Up arrow
                    self.arduino.move_relative(0, -5)
                elif key == 84:  # Down arrow
                    self.arduino.move_relative(0, 5)
                elif key == 81:  # Left arrow
                    self.arduino.move_relative(-5, 0)
                elif key == 83:  # Right arrow
                    self.arduino.move_relative(5, 0)
                elif key == ord('h'):  # Hough params
                    self.show_hough_params()
                elif key == ord('t'):  # Test pattern
                    self.arduino.test_movement_pattern()
                elif key == ord('c'):  # Center
                    self.arduino.center_position()
                
                # Son hedef merkezini güncelle
                if best_dart:
                    last_target_center = best_dart['center']
                
        except KeyboardInterrupt:
            print("\\n⏹️ Test kullanıcı tarafından durduruldu")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.arduino.disconnect()
    
    def handle_targeting(self, best_dart, last_target_center):
        """Otomatik hedefleme mantığı"""
        
        if best_dart:
            current_center = best_dart['center']
            confidence = best_dart['confidence']
            
            # Kararlılık kontrolü
            is_stable = True
            if last_target_center:
                distance = np.sqrt((current_center[0] - last_target_center[0])**2 + 
                                 (current_center[1] - last_target_center[1])**2)
                is_stable = distance < 30  # Kararlılık eşiği
            
            if is_stable and confidence >= 0.6:
                if not self.target_locked:
                    # Yeni hedef kilitlendi
                    self.target_locked = True
                    self.lock_start_time = time.time()
                    print(f"🔒 Hedef kilitlendi: ({current_center[0]}, {current_center[1]}) Güven: {confidence:.2f}")
                
                # Kilitlenme süresini kontrol et
                lock_time = time.time() - self.lock_start_time
                
                if lock_time >= self.lock_duration:
                    # Ateş et!
                    self.execute_targeting(best_dart)
                    self.target_locked = False
                    self.lock_start_time = None
                    
            else:
                # Hedef kararsız
                if self.target_locked:
                    print("⚠️ Hedef kararsız, kilit kaldırıldı")
                    self.target_locked = False
                    self.lock_start_time = None
        else:
            # Hedef yok
            self.target_locked = False
            self.lock_start_time = None
    
    def execute_targeting(self, dart_info):
        """Hedefleme ve ateşleme"""
        center_x, center_y = dart_info['center']
        confidence = dart_info['confidence']
        
        print(f"🎯 TARGET ACQUIRED: ({center_x}, {center_y}) Conf: {confidence:.2f}")
        
        # Servo pozisyonunu hesapla (basit kalibrasyon)
        frame_center_x, frame_center_y = 320, 240  # 640x480 merkezleri
        
        # Göreceli pozisyon
        delta_x = center_x - frame_center_x
        delta_y = center_y - frame_center_y
        
        # Servo açılarına çevir (kalibrasyon faktörleri)
        current_pan, current_tilt = self.arduino.get_position()
        
        # Pan hareketi (X ekseninde ters yönlü)
        target_pan = current_pan - (delta_x * 0.12)  # Kalibrasyon faktörü
        target_tilt = current_tilt + (delta_y * 0.10)  # Kalibrasyon faktörü
        
        # Limitleri kontrol et
        target_pan = max(0, min(180, target_pan))
        target_tilt = max(20, min(160, target_tilt))
        
        print(f"🔄 Servo hareketi: Pan={target_pan:.1f}°, Tilt={target_tilt:.1f}°")
        
        # Servo'yu hareket ettir
        success = self.arduino.move_to_position(target_pan, target_tilt, smooth=True)
        
        if success:
            # Lazer ateşle
            time.sleep(0.5)  # Servo sabitlenme bekle
            print("🔴 LAZER ATEŞLENİYOR!")
            self.arduino.fire_laser(2.0)
            
            # Başarı bilgisi
            print(f"✅ TARGET HIT! Pozisyon: ({center_x}, {center_y})")
            
            # Kısa bekle
            time.sleep(1.0)
        else:
            print("❌ Servo hareket hatası!")
    
    def draw_system_status(self, frame, detections, best_dart, process_time, frame_count):
        """Sistem durumunu frame üzerine çiz"""
        display_frame = frame.copy()
        
        # Tüm detection'ları çiz
        for i, detection in enumerate(detections):
            x, y, w, h = detection['bbox']
            center = detection['center']
            confidence = detection['confidence']
            
            # Renk seç
            if detection == best_dart:
                color = (0, 255, 0)      # Yeşil - hedef
                thickness = 3
            elif confidence >= 0.5:
                color = (0, 255, 255)    # Sarı - geçerli
                thickness = 2
            else:
                color = (0, 128, 255)    # Turuncu - düşük güven
                thickness = 1
            
            # Bounding box
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, thickness)
            
            # Merkez noktası
            cv2.circle(display_frame, center, 8, color, -1)
            
            # Güven değeri
            cv2.putText(display_frame, f"{confidence:.2f}", 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Hough Circle göster
            if detection.get('refined', False):
                radius = detection.get('hough_radius', 10)
                cv2.circle(display_frame, center, radius, (255, 0, 255), 2)
        
        # Crosshair (kamera merkezi)
        center_x, center_y = 320, 240
        cv2.line(display_frame, (center_x-30, center_y), (center_x+30, center_y), (255, 255, 255), 2)
        cv2.line(display_frame, (center_x, center_y-30), (center_x, center_y+30), (255, 255, 255), 2)
        
        # Sistem bilgileri
        arduino_status = self.arduino.get_status()
        pan, tilt = arduino_status['pan'], arduino_status['tilt']
        
        # Üst bilgiler
        info_lines = [
            f"Frame: {frame_count}  Process: {process_time:.1f}ms",
            f"Detections: {len(detections)}  Best: {'YES' if best_dart else 'NO'}",
            f"Arduino: Pan={pan:.1f}° Tilt={tilt:.1f}°",
            f"Target Lock: {'ACTIVE' if self.target_locked else 'SEARCHING'}"
        ]
        
        for i, line in enumerate(info_lines):
            cv2.putText(display_frame, line, (10, 30 + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Kilitlenme durumu
        if self.target_locked and self.lock_start_time:
            remaining = max(0, self.lock_duration - (time.time() - self.lock_start_time))
            cv2.putText(display_frame, f"FIRING IN: {remaining:.1f}s", 
                       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return display_frame
    
    def show_hough_params(self):
        """Mevcut Hough parametrelerini göster"""
        params = self.dart_detector.get_hough_params()
        print("🔧 Mevcut Hough Circle Parametreleri:")
        for key, value in params.items():
            print(f"  {key}: {value}")


def main():
    """Ana test menüsü"""
    
    print("🎯 BARLAS Test Menüsü")
    print("=" * 40)
    print("1️⃣  Arduino Simülatör ile Tam Sistem")
    print("2️⃣  Sadece Dart Detection Test")
    print("3️⃣  Hough Circle Parameter Tuning")
    print("4️⃣  Arduino Simülatör Test")
    print("0️⃣  Çıkış")
    print("=" * 40)
    
    while True:
        choice = input("\\n🚀 Seçiminizi yapın (0-4): ").strip()
        
        if choice == "1":
            # Tam sistem testi
            print("🔥 Tam sistem testi başlatılıyor...")
            test_suite = BarlasInteractiveTest()
            test_suite.run_full_system_test()
            break
            
        elif choice == "2":
            # Sadece dart detection
            print("🎯 Dart detection testi başlatılıyor...")
            os.system("python test_yolo_hough.py")
            break
            
        elif choice == "3":
            # Hough tuning
            print("🔧 Hough Circle parameter tuning başlatılıyor...")
            os.system("python hough_tuner.py")
            break
            
        elif choice == "4":
            # Arduino simülatör
            print("🎮 Arduino simülatör testi başlatılıyor...")
            os.system("python arduino_simulator.py")
            break
            
        elif choice == "0":
            print("👋 Çıkılıyor...")
            break
            
        else:
            print("❌ Geçersiz seçim!")


if __name__ == "__main__":
    main()
