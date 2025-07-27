"""
BARLAS Arduino Simulation Test
Arduino olmadan sistem testi - kameradaki dart'ları tespit eder
ve Arduino komutlarını simüle eder
"""
import cv2
import numpy as np
import time
import math
from typing import Optional, Tuple

# Dart detector'ı import et
from dart_detector import DartDetector


class ArduinoSimulator:
    """
    Arduino Pan-Tilt simülatörü
    Gerçek Arduino olmadan sistem testi için
    """
    
    def __init__(self):
        self.pan_position = 90
        self.tilt_position = 90
        self.laser_active = False
        self.is_connected = True  # Simülasyon için her zaman True
        
        # Kalibrasyon
        self.calibration_offset_x = 0
        self.calibration_offset_y = 0
        
        print("[ArduinoSim] 🎮 Arduino Simülatörü başlatıldı")
    
    def move_to_position(self, pan_angle: float, tilt_angle: float) -> bool:
        """Simüle edilmiş servo hareketi"""
        # Sınırları kontrol et
        pan_angle = max(10, min(170, pan_angle))
        tilt_angle = max(30, min(150, tilt_angle))
        
        self.pan_position = pan_angle
        self.tilt_position = tilt_angle
        
        print(f"[ArduinoSim] 🔄 SERVO KOMUTU: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
        return True
    
    def enable_laser(self) -> bool:
        """Simüle edilmiş lazer açma"""
        self.laser_active = True
        print("[ArduinoSim] 🔴 LAZER AÇIK (Simülasyon)")
        return True
    
    def disable_laser(self) -> bool:
        """Simüle edilmiş lazer kapama"""
        self.laser_active = False
        print("[ArduinoSim] ⚫ Lazer kapalı (Simülasyon)")
        return True
    
    def center_position(self) -> bool:
        """Merkez pozisyon"""
        return self.move_to_position(90, 90)
    
    def pixel_to_angle(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> Tuple[float, float]:
        """Piksel koordinatlarını servo açılarına çevir"""
        # Kamera merkezi
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # Offset uygula
        offset_x = pixel_x - center_x + self.calibration_offset_x
        offset_y = pixel_y - center_y + self.calibration_offset_y
        
        # Kamera FOV (60° yatay, 45° dikey)
        horizontal_fov = 60
        vertical_fov = 45
        
        # Açı hesapla
        pan_adjustment = (offset_x / center_x) * (horizontal_fov / 2)
        tilt_adjustment = -(offset_y / center_y) * (vertical_fov / 2)
        
        target_pan = self.pan_position + pan_adjustment
        target_tilt = self.tilt_position + tilt_adjustment
        
        return target_pan, target_tilt
    
    def aim_at_pixel(self, pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> bool:
        """Piksele nişan al"""
        target_pan, target_tilt = self.pixel_to_angle(pixel_x, pixel_y, frame_width, frame_height)
        
        print(f"[ArduinoSim] 🎯 HEDEF: Piksel ({pixel_x}, {pixel_y}) -> Servo ({target_pan:.1f}°, {target_tilt:.1f}°)")
        
        if self.move_to_position(target_pan, target_tilt):
            return self.enable_laser()
        return False


class DartTargetingSimulator:
    """
    Dart hedefleme simülatörü - Arduino'suz test
    """
    
    def __init__(self, camera_index=0):
        print("[DartSim] 🎯 Dart Hedefleme Simülatörü başlatılıyor...")
        
        # Simüle edilmiş Arduino
        self.arduino_sim = ArduinoSimulator()
        
        # Dart detector
        try:
            self.dart_detector = DartDetector(confidence_threshold=0.5)
            print("[DartSim] ✅ Dart detector hazır")
        except Exception as e:
            print(f"[DartSim] ⚠️ YOLO yok, dummy detector kullanılıyor: {e}")
            self.dart_detector = None
        
        # Kamera
        self.camera_index = camera_index
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480
        
        # Hedefleme durumu
        self.current_target = None
        self.target_lock_time = 0
        self.targeting_start_time = 0
        self.lock_duration = 2.0  # 2 saniye kilitlenme
        
        print("[DartSim] 🎮 Simülatör hazır!")
    
    def initialize_camera(self):
        """Kamerayı başlat"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"[DartSim] ❌ Kamera {self.camera_index} açılamadı!")
                return False
            
            # Kamera ayarları
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            # Gerçek çözünürlük
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[DartSim] ✅ Kamera: {self.frame_width}x{self.frame_height}")
            return True
            
        except Exception as e:
            print(f"[DartSim] ❌ Kamera hatası: {e}")
            return False
    
    def detect_objects_simple(self, frame):
        """Basit nesne tespiti (YOLO yoksa)"""
        # Yeşil nesneleri tespit et (dart yerine)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Yeşil renk aralığı
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Kontür bul
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum alan
                x, y, w, h = cv2.boundingRect(contour)
                
                detection = {
                    'bbox': [x, y, w, h],
                    'center': (x + w//2, y + h//2),
                    'confidence': 0.8,  # Sabit güven
                    'area': area
                }
                detections.append(detection)
        
        return detections
    
    def run_simulation(self):
        """Ana simülasyon döngüsü"""
        
        if not self.initialize_camera():
            return
        
        print("\n🚀 DART HEDEFLEMe SİMÜLASYONU BAŞLIYOR!")
        print("=" * 50)
        print("📋 Kontroller:")
        print("  'q' - Çıkış")
        print("  'space' - Manuel lazer açma/kapama")
        print("  'c' - Merkez pozisyon")
        print("  'wasd' - Manuel servo hareket")
        print("  Mouse - Manuel hedefe tıklama")
        print("=" * 50)
        print("🎯 Kameraya yeşil nesne gösterin (dart yerine)")
        print()
        
        # Mouse callback
        cv2.namedWindow('BARLAS Dart Targeting Simulation')
        cv2.setMouseCallback('BARLAS Dart Targeting Simulation', self.mouse_callback)
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            try:
                current_time = time.time()
                
                # Nesne tespiti
                if self.dart_detector:
                    detections = self.dart_detector.detect_darts(frame)
                else:
                    detections = self.detect_objects_simple(frame)
                
                # En iyi hedefi seç
                if detections:
                    best_detection = max(detections, key=lambda d: d['confidence'] * d['area'])
                    target_x, target_y = best_detection['center']
                    
                    # Hedef değişti mi?
                    if self.current_target is None:
                        self.current_target = (target_x, target_y)
                        self.targeting_start_time = current_time
                        print(f"[DartSim] 🎯 YENİ HEDEF TESPIT EDİLDİ: ({target_x}, {target_y})")
                    
                    else:
                        # Hedef kararlılık kontrolü
                        prev_x, prev_y = self.current_target
                        distance = math.sqrt((target_x - prev_x)**2 + (target_y - prev_y)**2)
                        
                        if distance < 30:  # Kararlı hedef
                            lock_time = current_time - self.targeting_start_time
                            
                            # Kilitlenme süresi doldu mu?
                            if lock_time >= self.lock_duration:
                                print(f"[DartSim] 🔥 HEDEF KİLİTLENDİ! Arduino'ya komut gönderiliyor...")
                                self.arduino_sim.aim_at_pixel(target_x, target_y, self.frame_width, self.frame_height)
                                
                                # Yeni hedef ara
                                self.current_target = None
                                self.targeting_start_time = 0
                        
                        else:
                            # Hedef değişti
                            self.current_target = (target_x, target_y)
                            self.targeting_start_time = current_time
                            print(f"[DartSim] 🔄 Hedef değişti: ({target_x}, {target_y})")
                
                else:
                    # Hedef yok
                    if self.current_target:
                        print("[DartSim] ❌ Hedef kaybedildi")
                        self.current_target = None
                        self.arduino_sim.disable_laser()
                
                # Görsel çizim
                display_frame = self.draw_simulation_info(frame, detections, current_time)
                cv2.imshow('BARLAS Dart Targeting Simulation', display_frame)
                
                # Klavye kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    if self.arduino_sim.laser_active:
                        self.arduino_sim.disable_laser()
                    else:
                        self.arduino_sim.enable_laser()
                elif key == ord('c'):
                    self.arduino_sim.center_position()
                elif key == ord('w'):
                    self.arduino_sim.move_to_position(
                        self.arduino_sim.pan_position,
                        self.arduino_sim.tilt_position - 5
                    )
                elif key == ord('s'):
                    self.arduino_sim.move_to_position(
                        self.arduino_sim.pan_position,
                        self.arduino_sim.tilt_position + 5
                    )
                elif key == ord('a'):
                    self.arduino_sim.move_to_position(
                        self.arduino_sim.pan_position - 5,
                        self.arduino_sim.tilt_position
                    )
                elif key == ord('d'):
                    self.arduino_sim.move_to_position(
                        self.arduino_sim.pan_position + 5,
                        self.arduino_sim.tilt_position
                    )
                
            except Exception as e:
                print(f"[DartSim] Simülasyon hatası: {e}")
                time.sleep(0.1)
        
        # Temizlik
        self.cap.release()
        cv2.destroyAllWindows()
        print("\n🏁 Simülasyon sonlandırıldı")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Mouse ile manuel hedefleme"""
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"[DartSim] 🖱️ Mouse hedefi: ({x}, {y})")
            self.arduino_sim.aim_at_pixel(x, y, self.frame_width, self.frame_height)
    
    def draw_simulation_info(self, frame, detections, current_time):
        """Simülasyon bilgilerini çiz"""
        display_frame = frame.copy()
        
        # Tespitleri çiz
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            center_x, center_y = detection['center']
            
            # Bounding box
            color = (0, 255, 0) if confidence >= 0.5 else (0, 165, 255)
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, 2)
            
            # Merkez noktası
            cv2.circle(display_frame, (center_x, center_y), 5, color, -1)
            
            # Güven skoru
            cv2.putText(display_frame, f"Obj: {confidence:.2f}", (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Mevcut hedef
        if self.current_target:
            target_x, target_y = self.current_target
            
            # Hedef çizgisi
            cv2.circle(display_frame, (target_x, target_y), 20, (255, 255, 0), 3)
            cv2.circle(display_frame, (target_x, target_y), 8, (255, 255, 0), -1)
            
            # Kilitlenme progress
            if self.targeting_start_time > 0:
                elapsed = current_time - self.targeting_start_time
                progress = min(elapsed / self.lock_duration, 1.0)
                bar_width = int(80 * progress)
                
                # Progress bar
                cv2.rectangle(display_frame, (target_x - 40, target_y - 35),
                             (target_x - 40 + bar_width, target_y - 30), (0, 255, 255), -1)
                cv2.rectangle(display_frame, (target_x - 40, target_y - 35),
                             (target_x + 40, target_y - 30), (255, 255, 255), 2)
                
                # Metin
                status = "LOCKED!" if progress >= 1.0 else f"LOCKING {progress*100:.0f}%"
                cv2.putText(display_frame, status, (target_x - 50, target_y - 45),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Arduino durumu
        arduino_info = [
            f"Arduino Sim: Pan={self.arduino_sim.pan_position:.0f}°, Tilt={self.arduino_sim.tilt_position:.0f}°",
            f"Laser: {'ON' if self.arduino_sim.laser_active else 'OFF'}",
            f"Target: {self.current_target if self.current_target else 'None'}"
        ]
        
        for i, info in enumerate(arduino_info):
            cv2.putText(display_frame, info, (10, 30 + i*25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Sistem durumu
        status_color = (0, 255, 0) if self.arduino_sim.laser_active else (255, 255, 255)
        status_text = "LAZER AKTIF (SIM)" if self.arduino_sim.laser_active else "HEDEF ARANIYOR"
        cv2.putText(display_frame, status_text, (10, display_frame.shape[0] - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Frame merkezi
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        cv2.circle(display_frame, (center_x, center_y), 3, (255, 0, 0), -1)
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 1)
        
        # Kontrol bilgileri
        controls = ["Q:Quit", "Space:Laser", "C:Center", "WASD:Move", "Mouse:Aim"]
        for i, control in enumerate(controls):
            cv2.putText(display_frame, control, (display_frame.shape[1] - 200, 30 + i*20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        return display_frame


def main():
    """Ana test fonksiyonu"""
    
    print("🎮 BARLAS ARDUINO DART TARGETING SIMULATION")
    print("=" * 60)
    print("Arduino olmadan sistem testi")
    print("Kameradaki nesneleri tespit eder ve Arduino komutlarını simüle eder")
    print("=" * 60)
    
    try:
        # Simülatör oluştur
        simulator = DartTargetingSimulator(camera_index=0)
        
        # Simülasyonu başlat
        simulator.run_simulation()
        
    except Exception as e:
        print(f"❌ Simülasyon hatası: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
