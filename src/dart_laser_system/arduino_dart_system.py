"""
BARLAS Arduino Dart Laser Targeting System
Arduino Pan-Tilt ile dart hedefleme sistemi
"""
import cv2
import numpy as np
import time
import threading
import math
from typing import Dict, List, Tuple, Optional

from dart_detector import DartDetector
from arduino_controller import ArduinoPanTiltController


class ArduinoDartLaserSystem:
    """
    Arduino tabanlı dart lazer hedefleme sistemi
    YOLO dart detection + Arduino servo kontrolü
    """
    
    def __init__(self, camera_index=0, arduino_port='COM3'):
        """
        Arduino Dart Laser System
        
        Args:
            camera_index: Kamera indeksi
            arduino_port: Arduino serial port
        """
        
        print("[ArduinoDartSystem] Sistem başlatılıyor...")
        
        # Dart detector
        try:
            self.dart_detector = DartDetector(confidence_threshold=0.6)
            print("[ArduinoDartSystem] ✅ Dart detector hazır")
        except Exception as e:
            print(f"[ArduinoDartSystem] ❌ Dart detector hatası: {e}")
            raise
        
        # Arduino Pan-Tilt kontrolcüsü
        try:
            self.arduino_controller = ArduinoPanTiltController(port=arduino_port)
            if not self.arduino_controller.is_connected:
                raise Exception("Arduino bağlanamadı")
            print("[ArduinoDartSystem] ✅ Arduino Pan-Tilt hazır")
        except Exception as e:
            print(f"[ArduinoDartSystem] ❌ Arduino hatası: {e}")
            raise
        
        # Kamera kurulumu
        self.camera_index = camera_index
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480
        
        # Sistem durumu
        self.is_running = False
        self.is_targeting = False
        self.current_target = None
        self.target_lock_duration = 0.0
        
        # Threading
        self.targeting_thread = None
        
        # Hedefleme parametreleri
        self.target_confidence_threshold = 0.6
        self.target_lock_time = 2.0
        self.laser_pulse_duration = 5.0
        self.stability_threshold = 30
        
        print("[ArduinoDartSystem] 🎯 Sistem hazır - hedefleme başlayabilir")
    
    def initialize_camera(self):
        """Kamerayı başlat"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"[ArduinoDartSystem] ❌ Kamera {self.camera_index} açılamadı!")
                return False
            
            # Kamera ayarları
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Gerçek çözünürlük
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[ArduinoDartSystem] ✅ Kamera: {self.frame_width}x{self.frame_height}")
            return True
            
        except Exception as e:
            print(f"[ArduinoDartSystem] Kamera hatası: {e}")
            return False
    
    def start_targeting_system(self):
        """Hedefleme sistemini başlat"""
        
        if not self.initialize_camera():
            return False
        
        self.is_running = True
        self.targeting_thread = threading.Thread(target=self._targeting_loop, daemon=True)
        self.targeting_thread.start()
        
        print("[ArduinoDartSystem] 🚀 Hedefleme sistemi başlatıldı")
        return True
    
    def stop_targeting_system(self):
        """Hedefleme sistemini durdur"""
        
        print("[ArduinoDartSystem] Sistem durduruluyor...")
        
        self.is_running = False
        self.is_targeting = False
        
        if self.targeting_thread:
            self.targeting_thread.join(timeout=3.0)
        
        if self.cap:
            self.cap.release()
        
        # Arduino'yu merkeze getir
        self.arduino_controller.center_position()
        self.arduino_controller.disable_laser()
        
        cv2.destroyAllWindows()
        print("[ArduinoDartSystem] ⚫ Sistem durduruldu")
    
    def _targeting_loop(self):
        """Ana hedefleme döngüsü"""
        
        print("[ArduinoDartSystem] 👁️ Hedefleme döngüsü başladı")
        
        last_target_time = 0
        laser_end_time = 0
        previous_center = None
        
        while self.is_running and self.cap and self.cap.isOpened():
            
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            try:
                # Dart tespiti
                detections = self.dart_detector.detect_darts(frame)
                
                current_time = time.time()
                
                if detections:
                    # Kararlı dart seç
                    stable_dart = self.dart_detector.get_stable_dart(
                        detections, previous_center, self.stability_threshold)
                    
                    if stable_dart:
                        target_x, target_y = stable_dart['center']
                        confidence = stable_dart['confidence']
                        
                        # Hedef kararlılık kontrolü
                        if self.current_target is None:
                            self.current_target = (target_x, target_y)
                            previous_center = (target_x, target_y)
                            last_target_time = current_time
                            self.target_lock_duration = 0
                            
                            print(f"[ArduinoDartSystem] 🎯 YENİ HEDEF: ({target_x}, {target_y}), Güven: {confidence:.2f}")
                        
                        else:
                            # Hedef kararlılık testi
                            prev_x, prev_y = self.current_target
                            distance = math.sqrt((target_x - prev_x)**2 + (target_y - prev_y)**2)
                            
                            if distance < self.stability_threshold:
                                # Hedef stabil
                                self.target_lock_duration = current_time - last_target_time
                                previous_center = (target_x, target_y)
                                
                                # Kilitlenme süresi doldu mu?
                                if (self.target_lock_duration >= self.target_lock_time and 
                                    not self.is_targeting and 
                                    current_time > laser_end_time):
                                    
                                    self._fire_laser_at_target(target_x, target_y)
                                    laser_end_time = current_time + self.laser_pulse_duration
                            
                            else:
                                # Hedef değişti
                                self.current_target = (target_x, target_y)
                                previous_center = (target_x, target_y)
                                last_target_time = current_time
                                self.target_lock_duration = 0
                                
                                print(f"[ArduinoDartSystem] 🔄 Hedef değişti: ({target_x}, {target_y})")
                
                else:
                    # Dart bulunamadı
                    if self.current_target is not None:
                        print("[ArduinoDartSystem] ❌ Hedef kaybedildi")
                        self.current_target = None
                        previous_center = None
                        self.target_lock_duration = 0
                        
                        # Lazer kapat
                        if self.is_targeting:
                            self.arduino_controller.disable_laser()
                            self.is_targeting = False
                
                # Lazer süre kontrolü
                if self.is_targeting and current_time > laser_end_time:
                    self.arduino_controller.disable_laser()
                    self.is_targeting = False
                    print("[ArduinoDartSystem] ⏰ Lazer süresi doldu")
                
                # Görsel çıktı
                display_frame = self._draw_targeting_info(frame, detections)
                cv2.imshow('BARLAS Arduino Dart Laser Targeting', display_frame)
                
                # Klavye kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.is_running = False
                    break
                elif key == ord(' '):
                    # Manuel lazer
                    if self.arduino_controller.laser_active:
                        self.arduino_controller.disable_laser()
                    else:
                        self.arduino_controller.enable_laser()
                elif key == ord('c'):
                    # Merkez
                    self.arduino_controller.center_position()
                elif key == ord('w'):
                    # Tilt up
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position,
                        self.arduino_controller.tilt_position - 5
                    )
                elif key == ord('s'):
                    # Tilt down
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position,
                        self.arduino_controller.tilt_position + 5
                    )
                elif key == ord('a'):
                    # Pan left
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position - 5,
                        self.arduino_controller.tilt_position
                    )
                elif key == ord('d'):
                    # Pan right
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position + 5,
                        self.arduino_controller.tilt_position
                    )
                
            except Exception as e:
                print(f"[ArduinoDartSystem] Döngü hatası: {e}")
                time.sleep(0.1)
            
            time.sleep(0.033)  # ~30 FPS
    
    def _fire_laser_at_target(self, target_x, target_y):
        """Hedefe lazer ateşle"""
        
        print(f"[ArduinoDartSystem] 🔥 LAZER ATEŞİ: ({target_x}, {target_y})")
        
        try:
            # Arduino ile hedefe nişan al
            if self.arduino_controller.aim_at_pixel(target_x, target_y, self.frame_width, self.frame_height):
                self.is_targeting = True
                print("[ArduinoDartSystem] 🎯 HEDEF KİLİTLENDİ - LAZER AKTİF")
            else:
                print("[ArduinoDartSystem] ❌ Arduino lazer ateş hatası")
            
        except Exception as e:
            print(f"[ArduinoDartSystem] Lazer ateş hatası: {e}")
    
    def _draw_targeting_info(self, frame, detections):
        """Hedefleme bilgilerini çiz"""
        
        # Dart tespitlerini çiz
        highlight_dart = None
        if self.current_target and detections:
            tx, ty = self.current_target
            min_dist = float('inf')
            
            for detection in detections:
                dx, dy = detection['center']
                dist = math.sqrt((dx - tx)**2 + (dy - ty)**2)
                if dist < min_dist:
                    min_dist = dist
                    highlight_dart = detection
        
        display_frame = self.dart_detector.draw_detections(frame, detections, highlight_dart)
        
        # Mevcut hedef
        if self.current_target:
            target_x, target_y = self.current_target
            
            # Hedef merkezi
            cv2.circle(display_frame, (target_x, target_y), 15, (255, 255, 0), 3)
            cv2.circle(display_frame, (target_x, target_y), 5, (255, 255, 0), -1)
            
            # Kilitlenme progress bar
            if self.target_lock_duration > 0:
                lock_progress = min(self.target_lock_duration / self.target_lock_time, 1.0)
                bar_width = int(100 * lock_progress)
                
                cv2.rectangle(display_frame, (target_x - 50, target_y - 30), 
                             (target_x - 50 + bar_width, target_y - 25), (0, 255, 255), -1)
                cv2.rectangle(display_frame, (target_x - 50, target_y - 30), 
                             (target_x + 50, target_y - 25), (255, 255, 255), 2)
                
                # Kilitlenme metni
                if lock_progress >= 1.0:
                    cv2.putText(display_frame, "LOCKED", (target_x - 30, target_y - 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, f"LOCKING {lock_progress*100:.0f}%", 
                               (target_x - 50, target_y - 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Sistem durumu
        status_color = (0, 255, 0) if self.is_targeting else (255, 255, 255)
        laser_status = "LAZER AKTİF" if self.is_targeting else "HEDEF ARANIYOṘ"
        
        cv2.putText(display_frame, laser_status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Arduino durumu
        arduino_status = f"Arduino: Pan={self.arduino_controller.pan_position:.0f}° Tilt={self.arduino_controller.tilt_position:.0f}°"
        cv2.putText(display_frame, arduino_status, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Kontroller
        controls = [
            "'q': Quit", "'space': Toggle Laser", "'c': Center",
            "'wasd': Manual Pan/Tilt"
        ]
        for i, control in enumerate(controls):
            cv2.putText(display_frame, control, (display_frame.shape[1] - 250, 30 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Frame merkezi
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        cv2.circle(display_frame, (center_x, center_y), 3, (255, 0, 0), -1)
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 1)
        
        return display_frame
    
    def set_targeting_parameters(self, confidence_threshold=0.6, lock_time=2.0, 
                               laser_duration=5.0, stability_threshold=30):
        """Hedefleme parametrelerini ayarla"""
        self.target_confidence_threshold = confidence_threshold
        self.target_lock_time = lock_time
        self.laser_pulse_duration = laser_duration
        self.stability_threshold = stability_threshold
        
        # Dart detector'ı güncelle
        self.dart_detector.set_confidence_threshold(confidence_threshold)
        
        print(f"[ArduinoDartSystem] Parametreler güncellendi:")
        print(f"  Güven eşiği: {confidence_threshold}")
        print(f"  Kilitlenme süresi: {lock_time}s")
        print(f"  Lazer süresi: {laser_duration}s")
        print(f"  Kararlılık eşiği: {stability_threshold}px")
    
    def calibrate_system(self):
        """Sistem kalibrasyon modu"""
        print("\n🎯 SİSTEM KALİBRASYON MODU")
        print("=" * 40)
        print("1. Arduino servo'ları manuel olarak kontrol edin")
        print("2. Kamera görüntüsünde lazer nokta pozisyonunu gözlemleyin")
        print("3. Offset değerlerini ayarlayın")
        
        if self.initialize_camera():
            
            offset_x = 0
            offset_y = 0
            
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue
                
                # Mevcut offset'i göster
                cv2.putText(frame, f"Offset X: {offset_x}, Y: {offset_y}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.putText(frame, "Kontroller: WASD=Move, +=OffsetX+, -=OffsetX-, U=OffsetY+, J=OffsetY-, Q=Quit", 
                           (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # Frame merkezi
                center_x = frame.shape[1] // 2
                center_y = frame.shape[0] // 2
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.line(frame, (center_x - 30, center_y), (center_x + 30, center_y), (0, 0, 255), 2)
                cv2.line(frame, (center_x, center_y - 30), (center_x, center_y + 30), (0, 0, 255), 2)
                
                cv2.imshow('Arduino Sistem Kalibrasyon', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('w'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position,
                        self.arduino_controller.tilt_position - 5
                    )
                elif key == ord('s'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position,
                        self.arduino_controller.tilt_position + 5
                    )
                elif key == ord('a'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position - 5,
                        self.arduino_controller.tilt_position
                    )
                elif key == ord('d'):
                    self.arduino_controller.move_to_position(
                        self.arduino_controller.pan_position + 5,
                        self.arduino_controller.tilt_position
                    )
                elif key == ord(' '):
                    if self.arduino_controller.laser_active:
                        self.arduino_controller.disable_laser()
                    else:
                        self.arduino_controller.enable_laser()
                elif key == ord('c'):
                    self.arduino_controller.center_position()
                elif key == ord('+') or key == ord('='):
                    offset_x += 5
                    self.arduino_controller.calibrate_offset(offset_x, offset_y)
                elif key == ord('-'):
                    offset_x -= 5
                    self.arduino_controller.calibrate_offset(offset_x, offset_y)
                elif key == ord('u'):
                    offset_y -= 5
                    self.arduino_controller.calibrate_offset(offset_x, offset_y)
                elif key == ord('j'):
                    offset_y += 5
                    self.arduino_controller.calibrate_offset(offset_x, offset_y)
            
            self.cap.release()
            cv2.destroyAllWindows()
    
    def cleanup(self):
        """Sistem temizliği"""
        self.stop_targeting_system()
        self.arduino_controller.cleanup()
        print("[ArduinoDartSystem] Sistem temizliği tamamlandı")


def main():
    """Ana program"""
    
    print("=" * 60)
    print("🎯 BARLAS ARDUINO DART LASER TARGETING SYSTEM 🎯")
    print("=" * 60)
    
    # Port seçimi
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        print("Mevcut COM portları:")
        for port, desc, hwid in sorted(ports):
            print(f"  {port}: {desc}")
        print()
    except:
        pass
    
    arduino_port = input("Arduino port (varsayılan COM3): ").strip() or "COM3"
    
    try:
        # Sistem oluştur
        dart_system = ArduinoDartLaserSystem(camera_index=0, arduino_port=arduino_port)
        
        print("\n🚀 Sistem başlatılıyor...")
        
        # Parametreleri ayarla
        dart_system.set_targeting_parameters(
            confidence_threshold=0.5,
            lock_time=1.5,
            laser_duration=3.0,
            stability_threshold=25
        )
        
        # Menü
        while True:
            print("\n" + "="*40)
            print("🎯 ANA MENÜ")
            print("="*40)
            print("1. Hedefleme Sistemi Başlat")
            print("2. Sistem Kalibrasyonu")
            print("3. Arduino Manuel Kontrol")
            print("4. Çıkış")
            
            choice = input("\nSeçiminiz (1-4): ").strip()
            
            if choice == '1':
                # Hedefleme sistemi
                if dart_system.start_targeting_system():
                    print("✅ Hedefleme sistemi aktif!")
                    print("\n📋 Kontroller:")
                    print("  'q' - Çıkış")
                    print("  'space' - Manuel lazer")
                    print("  'c' - Merkez")
                    print("  'wasd' - Manuel hareket")
                    print("\n🎯 Dart aramamaya başlıyor... Kameraya dart gösterin!")
                    
                    # Sistem çalışırken bekle
                    try:
                        while dart_system.is_running:
                            time.sleep(0.1)
                    except KeyboardInterrupt:
                        print("\n⚠️ Kullanıcı tarafından iptal edildi")
                
            elif choice == '2':
                # Kalibrasyon
                dart_system.calibrate_system()
                
            elif choice == '3':
                # Manuel kontrol
                dart_system.arduino_controller.manual_control()
                
            elif choice == '4':
                print("👋 Sistemden çıkış...")
                break
                
            else:
                print("❌ Geçersiz seçim!")
    
    except Exception as e:
        print(f"❌ Sistem hatası: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if 'dart_system' in locals():
            dart_system.cleanup()
        
        print("\n🏁 Program sonlandırıldı")


if __name__ == "__main__":
    main()
