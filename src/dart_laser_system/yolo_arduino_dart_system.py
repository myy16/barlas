#!/usr/bin/env python3
"""
BARLAS Arduino Dart Laser System - Gelişmiş UI Versiyonu
YOLO Dart Detection + Hough Circle + Arduino Pan-Tilt
Gerçek dart tanıma sistemi ile Arduino kontrolü
"""
import cv2
import numpy as np
import time
import math
import os
import sys
from typing import Optional, Tuple, List

# Mevcut modülleri import et
from dart_detector import DartDetector
from arduino_simulator import ArduinoSimulator

# Arduino Controller Import - Basitleştirilmiş
try:
    from arduino_controller_simple import ArduinoPanTiltController
    ARDUINO_AVAILABLE = True
    print("✅ Arduino Controller (Simple) modülü yüklendi!")
except ImportError:
    ARDUINO_AVAILABLE = False
    print("⚠️ Arduino Controller bulunamadı, simülatör kullanılacak")


class YOLOArduinoDartSystem:
    """
    YOLO + Hough Circle + Arduino Dart Laser System
    Gelişmiş UI ve kontrol özellikleri ile
    """
    
    def __init__(self, camera_index=0, arduino_port=None):
        """
        YOLO + Arduino Dart Targeting System
        
        Args:
            camera_index: Kamera index (0 = varsayılan)  
            arduino_port: Arduino port (None = otomatik, "/dev/ttyACM0" = manuel)
        """
        print("[YOLOArduinoSystem] 🎯 YOLO+Arduino Dart Sistem başlatılıyor...")
        
        # Manuel Arduino port
        self.manual_arduino_port = arduino_port
        
        # Dart Detector (mevcut optimize edilmiş sistemi kullan)
        try:
            self.dart_detector = DartDetector(confidence_threshold=0.5)
            
            # Hough Circle parametrelerini optimize et
            self.dart_detector.set_hough_params(
                param1=60,           # Canny edge threshold
                param2=20,           # Accumulator threshold (daha hassas)
                minDist_ratio=0.25,  # Daha yakın circle'lara izin ver
                minRadius_ratio=0.08,
                maxRadius_ratio=0.6
            )
            print("[YOLOArduinoSystem] ✅ Dart Detector yüklendi (optimize edilmiş parametreler)")
            
        except Exception as e:
            print(f"[YOLOArduinoSystem] ❌ Dart Detector hatası: {e}")
            raise
        
        # Arduino Controller - Gerçek veya Simülatör
        self.arduino_controller = self.initialize_arduino()
        
        # EĞER ARDUINO BAĞLANAMAZSA SİSTEMİ DURDUR
        if self.arduino_controller is None:
            print("[YOLOArduinoSystem] ❌ Arduino bağlantısı başarısız! Sistem durduruluyor.")
            raise Exception("Arduino bağlantısı zorunlu!")
        
        # BAŞLANGIÇTA LAZERİ KAPALI YÜKLÜYORUZ
        self.ensure_laser_off_at_startup()
        
        # Kamera
        self.camera_index = camera_index
        self.cap = None
        self.frame_width = 640   # Daha hızlı işlem için
        self.frame_height = 480  # Daha hızlı işlem için
        
        # Hedefleme durumu
        self.current_dart_target = None
        self.targeting_start_time = 0
        self.lock_duration = 1.0  # 1 saniye kilitlenme (daha hızlı)
        self.confidence_threshold = 0.5
        
        # UI durumu
        self.show_all_detections = True
        self.show_crosshair = True
        
        # Lazer durumu - BAŞLANGIÇTA KAPALI!
        self.laser_enabled_by_system = False
        
        print("[YOLOArduinoSystem] 🎮 Sistem hazır!")
        print("[YOLOArduinoSystem] ⚫ Lazer başlangıçta KAPALI - güvenlik önlemi")
    
    def initialize_arduino(self):
        """Arduino Controller başlat - Gerçek veya Simülatör"""
        if ARDUINO_AVAILABLE:
            # Manuel port belirtilmişse önce onu dene
            if self.manual_arduino_port:
                print(f"[YOLOArduinoSystem] 🎯 Manuel Arduino portu deneniyor: {self.manual_arduino_port}")
                
                # Windows'ta Linux path düzeltmesi
                port_to_try = self.manual_arduino_port
                if port_to_try == "/dev/ttyACM0" and sys.platform.startswith('win'):
                    port_to_try = "COM7"  # Windows'ta muhtemel port
                    print(f"[YOLOArduinoSystem] 🔧 Windows için port düzeltildi: {port_to_try}")
                
                try:
                    arduino = ArduinoPanTiltController(port=port_to_try, baud_rate=9600, timeout=2)
                    if arduino.connect():
                        print(f"[YOLOArduinoSystem] ✅ Manuel Arduino bağlandı: {port_to_try}")
                        return arduino
                    else:
                        arduino.disconnect()
                        print(f"[YOLOArduinoSystem] ❌ Manuel port bağlantısı başarısız: {port_to_try}")
                except Exception as e:
                    print(f"[YOLOArduinoSystem] ❌ Manuel port hatası: {e}")
            
            # Otomatik port tarama - İşletim sistemine göre
            import sys
            if sys.platform.startswith('linux'):
                ports_to_try = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
            else:
                # Windows - gerçek USB portları öncelikli
                ports_to_try = ['COM7', 'COM6', 'COM5', 'COM8', 'COM9', 'COM10']
            
            for port in ports_to_try:
                try:
                    print(f"[YOLOArduinoSystem] 🔍 Arduino bağlantısı deneniyor: {port}")
                    # Arduino basit versiyonu - daha hızlı bağlantı
                    arduino = ArduinoPanTiltController(port=port, baud_rate=9600, timeout=2)
                    if arduino.connect():
                        print(f"[YOLOArduinoSystem] ✅ Gerçek Arduino bağlandı: {port}")
                        return arduino
                    else:
                        print(f"[YOLOArduinoSystem] ❌ Port yanıt vermiyor: {port}")
                        arduino.disconnect()
                except Exception as e:
                    print(f"[YOLOArduinoSystem] ❌ Port hatası {port}: {e}")
                    continue
            
            print("[YOLOArduinoSystem] ❌ Gerçek Arduino bulunamadı!")
            print("[YOLOArduinoSystem] ⚠️ Simülatör kullanmak istiyor musunuz? (Y/n)")
            
            # Kullanıcıdan onay al
            try:
                import sys
                if sys.stdin.isatty():  # Terminal varsa kullanıcıdan sor
                    response = input("Simülatör kullanılsın mı? (Y/n): ").strip().lower()
                    if response in ['n', 'no', 'hayır']:
                        print("[YOLOArduinoSystem] ❌ Gerçek Arduino bağlantısı zorunlu. Sistem durduruluyor.")
                        return None
                else:
                    print("[YOLOArduinoSystem] ⚠️ Terminal yok, simülatör otomatik başlatılıyor")
            except:
                print("[YOLOArduinoSystem] ⚠️ Kullanıcı girişi alınamadı, simülatör başlatılıyor")
        
        # Simülatör kullan
        print("[YOLOArduinoSystem] 🎮 Arduino Simulator başlatılıyor...")
        return ArduinoSimulator()
    
    def ensure_laser_off_at_startup(self):
        """Sistem başlangıcında lazeri kapalı yap"""
        try:
            success = self.disable_laser()
            if success:
                print("[YOLOArduinoSystem] ✅ Başlangıç lazer durumu: KAPALI")
            else:
                print("[YOLOArduinoSystem] ⚠️ Lazer kapatma komutu gönderildi")
        except Exception as e:
            print(f"[YOLOArduinoSystem] ⚠️ Başlangıç lazer kontrolü: {e}")
    
    def initialize_camera(self):
        """Kamerayı başlat"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"[YOLOArduinoSystem] ❌ Kamera {self.camera_index} açılamadı!")
                return False
            
            # Kamera ayarları
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Gerçek çözünürlük
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[YOLOArduinoSystem] ✅ Kamera: {self.frame_width}x{self.frame_height}")
            return True
            
        except Exception as e:
            print(f"[YOLOArduinoSystem] ❌ Kamera hatası: {e}")
            return False
    
    def run_dart_targeting(self):
        """Ana dart hedefleme döngüsü"""
        
        if not self.initialize_camera():
            return
        
        print("\n🚀 YOLO+ARDUINO DART TARGETING BAŞLIYOR!")
        print("=" * 60)
        print("📋 Kontroller:")
        print("  'q' / ESC  - Çıkış")
        print("  'space'    - Manuel lazer açma/kapama") 
        print("  'c'        - Merkez pozisyon")
        print("  'wasd'     - Manuel servo hareket (±5°)")
        print("  '+/-'      - Güven eşiği ayarı")
        print("  'r'        - Reset hedefleme")
        print("  'f'        - Tüm tespitleri göster/gizle")
        print("  'x'        - Crosshair göster/gizle")
        print("  't'        - Tarama modu açma/kapama")
        print("  '1/2/3'    - Tarama hızı (yavaş/orta/hızlı)")
        print("  'Mouse'    - Manuel hedefleme")
        print("=" * 60)
        print("🎯 Kameraya DART gösterin!")
        print()
        
        # OpenCV penceresi
        window_name = 'BARLAS YOLO+Arduino Dart Targeting'
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        frame_count = 0
        fps_start_time = time.time()
        fps = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("[YOLOArduinoSystem] ⚠️ Kameradan görüntü alınamadı")
                    continue
                
                start_time = time.time()
                current_time = start_time
                
                try:
                    # Dart tespiti
                    detections = self.dart_detector.detect_darts(frame)
                    
                    # Güvenli dart'ları filtrele
                    valid_detections = [d for d in detections 
                                      if d['confidence'] >= self.confidence_threshold]
                    
                    best_dart = None
                    best_dart_center = None
                    
                    if valid_detections:
                        # En iyi dart'ı seç (güven * boyut)
                        best_dart = max(valid_detections, 
                                       key=lambda d: d['confidence'] * 
                                       (d['bbox'][2] * d['bbox'][3]))
                        
                        # Merkez koordinatını al
                        if 'refined_center' in best_dart and best_dart['refined_center']:
                            best_dart_center = best_dart['refined_center']
                        else:
                            # YOLO bbox merkezi
                            x, y, w, h = best_dart['bbox']
                            best_dart_center = (x + w//2, y + h//2)
                    
                    # Hedefleme logic
                    if best_dart_center:
                        target_x, target_y = best_dart_center
                        
                        # Yeni hedef mi?
                        if self.current_dart_target is None:
                            self.current_dart_target = best_dart_center
                            self.targeting_start_time = current_time
                            method = "Hough Circle" if best_dart.get('refined_center') else "YOLO BBox"
                            print(f"[YOLOArduinoSystem] 🎯 YENİ DART TESPIT: {method} - ({target_x}, {target_y}), Güven: {best_dart['confidence']:.2f}")
                        
                        else:
                            # Hedef kararlılık kontrolü
                            prev_x, prev_y = self.current_dart_target
                            distance = math.sqrt((target_x - prev_x)**2 + (target_y - prev_y)**2)
                            
                            if distance < 30:  # Kararlı hedef (30 piksel tolerans)
                                lock_time = current_time - self.targeting_start_time
                                
                                # Kilitlenme süresi doldu mu?
                                if lock_time >= self.lock_duration:
                                    # HEDEF KİLİTLENDİ - AMA LAZER HENÜZ AÇMA!
                                    if not self.laser_enabled_by_system:
                                        method = "Hough Circle" if best_dart.get('refined_center') else "YOLO BBox"
                                        print(f"[YOLOArduinoSystem] 🔥 DART KİLİTLENDİ! {method} Merkez: ({target_x}, {target_y})")
                                        print(f"[YOLOArduinoSystem] 🚀 Arduino'ya hedefleme komutu gönderiliyor...")
                                        
                                        # Arduino'ya hedefleme komutu gönder (LAZER AÇMADAN)
                                        success = self.aim_at_pixel_without_laser(target_x, target_y)
                                        if success:
                                            print("[YOLOArduinoSystem] ✅ Hedefleme başarılı! Merkez hizalamaya başlanıyor...")
                                        else:
                                            print("[YOLOArduinoSystem] ❌ Hedefleme hatası!")
                                
                                # HEDEF KİLİTLİ DURUMDA - SÜREKLİ TAKİP ET
                                self.current_dart_target = best_dart_center
                                
                                # KAMERA MERKEZİ HİZALAMA KONTROLÜ VE LAZER AKTİVASYONU
                                center_x = self.frame_width // 2
                                center_y = self.frame_height // 2
                                
                                # Merkez ile hedef arasındaki mesafe
                                center_distance = math.sqrt((target_x - center_x)**2 + (target_y - center_y)**2)
                                
                                # SADECE DART MERKEZDE OLDUĞUNDA LAZER AÇ!
                                if center_distance <= 10:  # 10 piksel tolerans
                                    if not self.laser_enabled_by_system:
                                        # İLK KEZ MERKEZE GELDİ - LAZER AÇ!
                                        if self.enable_laser():
                                            self.laser_enabled_by_system = True
                                            print(f"[YOLOArduinoSystem] 🔴 LAZER AKTİF - Hedef merkezde! Mesafe: {center_distance:.1f}px")
                                        else:
                                            print("[YOLOArduinoSystem] ⚠️ Lazer açılamadı")
                                    else:
                                        # ZATEN MERKEZDE VE LAZER AÇIK
                                        print(f"[YOLOArduinoSystem] ✅ HEDEF MERKEZDE KORUNUYOR! Mesafe: {center_distance:.1f}px")
                                
                                else:
                                    # HEDEF MERKEZ DEĞİL - LAZER KAPALI OLMALI
                                    if self.laser_enabled_by_system:
                                        # Merkez hizalama kaybedildi, lazeri kapat
                                        self.disable_laser()
                                        self.laser_enabled_by_system = False
                                        print(f"[YOLOArduinoSystem] ⚫ Lazer kapatıldı - hedef merkez dışına çıktı! Mesafe: {center_distance:.1f}px")
                                    
                                    # Merkez hizalama yap
                                    print(f"[YOLOArduinoSystem] 🎯 MERKEZ HİZALAMA: Hedef ({target_x},{target_y}) -> Merkez ({center_x},{center_y}), Mesafe: {center_distance:.1f}px")
                                    
                                    # Sürekli düzeltme yap
                                    correction_success = self.aim_at_pixel_without_laser(target_x, target_y)
                                    if correction_success:
                                        print("[YOLOArduinoSystem] ✅ Merkez düzeltmesi yapıldı")
                                    else:
                                        print("[YOLOArduinoSystem] ⚠️ Merkez düzeltmesi başarısız")
                            
                            else:
                                # Hedef değişti
                                self.current_dart_target = best_dart_center
                                self.targeting_start_time = current_time
                                print(f"[YOLOArduinoSystem] 🔄 Dart pozisyon değişti: ({target_x}, {target_y})")
                    
                    else:
                        # Dart bulunamadı
                        if self.current_dart_target:
                            print("[YOLOArduinoSystem] ❌ Dart kaybedildi")
                            self.current_dart_target = None
                            # HEDEF KAYBEDİLDİĞİNDE LAZERİ KAPAT
                            if self.laser_enabled_by_system:
                                self.disable_laser()
                                self.laser_enabled_by_system = False
                                print("[YOLOArduinoSystem] ⚫ Lazer kapatıldı - hedef kaybedildi")
                    
                    # FPS hesaplama
                    frame_count += 1
                    if frame_count % 10 == 0:
                        fps = 10 / (current_time - fps_start_time)
                        fps_start_time = current_time
                    
                    # İşlem süresi
                    process_time = (time.time() - start_time) * 1000
                    
                    # Görsel çizim
                    display_frame = self.draw_targeting_info(
                        frame, detections, valid_detections, 
                        best_dart, current_time, fps, process_time
                    )
                    
                    cv2.imshow(window_name, display_frame)
                    
                    # Klavye kontrolü
                    key = cv2.waitKey(1) & 0xFF
                    if not self.handle_keyboard(key):
                        break
                        
                except Exception as e:
                    print(f"[YOLOArduinoSystem] Döngü hatası: {e}")
                    time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n[YOLOArduinoSystem] Kullanıcı tarafından durduruldu")
        
        finally:
            # Temizlik
            self.cap.release()
            cv2.destroyAllWindows()
            # SİSTEM KAPATILIRKEN LAZERİ KAPAT
            if self.laser_enabled_by_system:
                self.disable_laser()
                print("[YOLOArduinoSystem] ⚫ Sistem kapatılırken lazer güvenli olarak kapatıldı")
            print("\n🏁 YOLO+Arduino Dart Targeting sonlandırıldı")
    
    def handle_keyboard(self, key) -> bool:
        """Klavye kontrollerini işle"""
        if key == ord('q') or key == 27:  # ESC
            return False
        elif key == ord(' '):
            # SPACE tuşu ile manuel lazer kontrolü
            if hasattr(self.arduino_controller, 'laser_active') and self.arduino_controller.laser_active:
                self.disable_laser()
                self.laser_enabled_by_system = False
                print("[YOLOArduinoSystem] ⚫ MANUEL LAZER KAPALI")
            else:
                if self.enable_laser():
                    self.laser_enabled_by_system = True
                    print("[YOLOArduinoSystem] 🔴 MANUEL LAZER AÇIK")
                else:
                    print("[YOLOArduinoSystem] ⚠️ Lazer açılamadı")
        elif key == ord('c'):
            if isinstance(self.arduino_controller, ArduinoSimulator):
                self.arduino_controller.stop_scanning()
            self.center_position()
            if isinstance(self.arduino_controller, ArduinoSimulator):
                self.arduino_controller.start_scanning()
        elif key == ord('r'):
            self.current_dart_target = None
            self.targeting_start_time = 0
            # RESET SIRASİNDA LAZERİ KAPAT
            if self.laser_enabled_by_system:
                self.disable_laser()
                self.laser_enabled_by_system = False
                print("[YOLOArduinoSystem] ⚫ Lazer kapatıldı - sistem resetlendi")
            # Taramaya devam et
            if isinstance(self.arduino_controller, ArduinoSimulator):
                self.arduino_controller.resume_scanning()
            print("[YOLOArduinoSystem] 🔄 Hedefleme resetlendi")
        elif key == ord('f'):
            self.show_all_detections = not self.show_all_detections
            status = "AÇIK" if self.show_all_detections else "KAPALI"
            print(f"[YOLOArduinoSystem] Tüm tespitler: {status}")
        elif key == ord('x'):
            self.show_crosshair = not self.show_crosshair
            status = "AÇIK" if self.show_crosshair else "KAPALI"
            print(f"[YOLOArduinoSystem] Crosshair: {status}")
        elif key == ord('w'):
            self.move_servo(0, -5)
        elif key == ord('s'):
            self.move_servo(0, 5)
        elif key == ord('a'):
            self.move_servo(-5, 0)
        elif key == ord('d'):
            self.move_servo(5, 0)
        elif key == ord('+') or key == ord('='):
            self.confidence_threshold = min(1.0, self.confidence_threshold + 0.05)
            print(f"[YOLOArduinoSystem] Güven eşiği: {self.confidence_threshold:.2f}")
        elif key == ord('-'):
            self.confidence_threshold = max(0.1, self.confidence_threshold - 0.05)
            print(f"[YOLOArduinoSystem] Güven eşiği: {self.confidence_threshold:.2f}")
        elif key == ord('t'):
            # Tarama modu açma/kapama
            if isinstance(self.arduino_controller, ArduinoSimulator):
                if self.arduino_controller.is_scanning:
                    self.arduino_controller.stop_scanning()
                    print("[YOLOArduinoSystem] 🔄 Tarama DURDURULDU")
                else:
                    self.arduino_controller.start_scanning()
                    print("[YOLOArduinoSystem] 🔄 Tarama BAŞLATILDI")
        elif key == ord('1'):
            # Tarama hızı - yavaş
            if isinstance(self.arduino_controller, ArduinoSimulator):
                self.arduino_controller.set_scanning_speed(2.0)
        elif key == ord('2'):
            # Tarama hızı - orta
            if isinstance(self.arduino_controller, ArduinoSimulator):
                self.arduino_controller.set_scanning_speed(5.0)
        elif key == ord('3'):
            # Tarama hızı - hızlı
            if isinstance(self.arduino_controller, ArduinoSimulator):
                self.arduino_controller.set_scanning_speed(10.0)
        
        return True
    
    def mouse_callback(self, event, x, y, flags, param):
        """Mouse ile manuel hedefleme"""
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"[YOLOArduinoSystem] 🖱️ Manuel hedef: ({x}, {y})")
            # MOUSE İLE MANUEL HEDEFLEMEEde LAZER AÇILMAZ
            success = self.aim_at_pixel_without_laser(x, y)
            if success:
                print("[YOLOArduinoSystem] ✅ Manuel hedefleme tamamlandı (lazer kapalı)")
            else:
                print("[YOLOArduinoSystem] ❌ Manuel hedefleme hatası")
    
    def aim_at_pixel_without_laser(self, pixel_x: int, pixel_y: int) -> bool:
        """Piksele nişan al - LAZER AÇMADAN"""
        try:
            # Simülatör kontrolü
            if isinstance(self.arduino_controller, ArduinoSimulator):
                # Simülatör için sadece pozisyon değiştir, lazer açma
                target_pan, target_tilt = self.arduino_controller.pixel_to_angle(
                    pixel_x, pixel_y, self.frame_width, self.frame_height)
                return self.arduino_controller.move_to_position(target_pan, target_tilt)
            
            # Gerçek Arduino kontrolü
            else:
                target_pan, target_tilt = self.pixel_to_servo_angle(pixel_x, pixel_y)
                return self.arduino_controller.move_to_position(target_pan, target_tilt)
                
        except Exception as e:
            print(f"[YOLOArduinoSystem] Hedefleme hatası: {e}")
            return False
    
    def aim_at_pixel(self, pixel_x: int, pixel_y: int) -> bool:
        """Piksele nişan al - LAZER İLE BİRLİKTE (sadece eski uyumluluk için)"""
        try:
            # Önce pozisyonu ayarla
            success = self.aim_at_pixel_without_laser(pixel_x, pixel_y)
            if success:
                # Sadece başarılı ise lazeri aç
                return self.enable_laser()
            return False
                
        except Exception as e:
            print(f"[YOLOArduinoSystem] Hedefleme hatası: {e}")
            return False
    
    def pixel_to_servo_angle(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """Piksel koordinatlarını servo açılarına çevir - ABSOLUTE pozisyon"""
        # Kamera merkezi
        center_x = self.frame_width / 2  # 320
        center_y = self.frame_height / 2  # 240
        
        # Offset hesapla (piksel farkı)
        offset_x = pixel_x - center_x
        offset_y = pixel_y - center_y
        
        # Kamera FOV - daha gerçekçi değerler
        horizontal_fov = 50  # Derece
        vertical_fov = 40    # Derece
        
        # Piksel başına açı
        degrees_per_pixel_x = horizontal_fov / self.frame_width
        degrees_per_pixel_y = vertical_fov / self.frame_height
        
        # MERKEZ REFERANSLI ABSOLUTE açı hesaplama
        # Merkez = 90°, sağ = +, sol = -, yukarı = +, aşağı = -
        target_pan = 90 + (offset_x * degrees_per_pixel_x)
        target_tilt = 90 - (offset_y * degrees_per_pixel_y)  # Y eksenini ters çevir
        
        print(f"[YOLOArduinoSystem] 🎯 Pixel({pixel_x},{pixel_y}) -> Servo({target_pan:.1f}°, {target_tilt:.1f}°)")
        
        # Sınırları kontrol et
        target_pan = max(10, min(170, target_pan))
        target_tilt = max(20, min(160, target_tilt))
        
        return target_pan, target_tilt
        
        # Sınırları kontrol et
        target_pan = max(10, min(170, target_pan))
        target_tilt = max(20, min(160, target_tilt))
        
        return target_pan, target_tilt
    
    def move_servo(self, pan_delta: float, tilt_delta: float):
        """Servo'yu relatif hareket ettir"""
        try:
            current_pan = getattr(self.arduino_controller, 'current_pan', 90)
            current_tilt = getattr(self.arduino_controller, 'current_tilt', 90)
            
            new_pan = max(10, min(170, current_pan + pan_delta))
            new_tilt = max(20, min(160, current_tilt + tilt_delta))
            
            self.arduino_controller.move_to_position(new_pan, new_tilt)
        except Exception as e:
            print(f"[YOLOArduinoSystem] Servo hareket hatası: {e}")
    
    def enable_laser(self) -> bool:
        """Lazer aç"""
        try:
            if hasattr(self.arduino_controller, 'enable_laser'):
                return self.arduino_controller.enable_laser()
            elif hasattr(self.arduino_controller, 'fire_laser'):
                return self.arduino_controller.fire_laser(duration=0.1)
            return False
        except Exception as e:
            print(f"[YOLOArduinoSystem] Lazer açma hatası: {e}")
            return False
    
    def disable_laser(self) -> bool:
        """Lazer kapat"""
        try:
            if hasattr(self.arduino_controller, 'disable_laser'):
                return self.arduino_controller.disable_laser()
            return True
        except Exception as e:
            print(f"[YOLOArduinoSystem] Lazer kapama hatası: {e}")
            return False
    
    def center_position(self) -> bool:
        """Merkez pozisyona git"""
        try:
            if hasattr(self.arduino_controller, 'center_position'):
                return self.arduino_controller.center_position()
            else:
                return self.arduino_controller.move_to_position(90, 90)
        except Exception as e:
            print(f"[YOLOArduinoSystem] Merkez pozisyon hatası: {e}")
            return False
    
    def draw_targeting_info(self, frame, detections, valid_detections, best_dart, current_time, fps, process_time):
        """Targeting bilgilerini çiz"""
        display_frame = frame.copy()
        
        # Tüm tespitleri çiz (opsiyonel)
        if self.show_all_detections:
            for detection in detections:
                x, y, w, h = detection['bbox']
                confidence = detection['confidence']
                
                # Renk seçimi
                if confidence >= self.confidence_threshold:
                    color = (0, 255, 0)  # Yeşil - geçerli
                    thickness = 2
                else:
                    color = (0, 0, 255)  # Kırmızı - düşük güven
                    thickness = 1
                
                # Bounding box
                cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, thickness)
                
                # Güven skoru
                cv2.putText(display_frame, f"{confidence:.2f}", (x, y-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # En iyi dart'ı vurgula
        if best_dart:
            x, y, w, h = best_dart['bbox']
            confidence = best_dart['confidence']
            
            # Best dart bounding box
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 255), 3)
            cv2.putText(display_frame, f"BEST: {confidence:.2f}", (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Merkez noktası
            if 'refined_center' in best_dart and best_dart['refined_center']:
                center_x, center_y = best_dart['refined_center']
                color = (255, 0, 255)  # Magenta - Hough Circle
                method = "Hough"
                
                # Circle çiz (eğer radius varsa)
                if 'refined_radius' in best_dart:
                    cv2.circle(display_frame, (center_x, center_y), 
                             best_dart['refined_radius'], color, 2)
            else:
                center_x = x + w//2
                center_y = y + h//2
                color = (255, 255, 0)  # Sarı - YOLO BBox
                method = "YOLO"
            
            # Merkez işareti
            cv2.circle(display_frame, (center_x, center_y), 5, color, -1)
            cv2.line(display_frame, (center_x-15, center_y), (center_x+15, center_y), color, 2)
            cv2.line(display_frame, (center_x, center_y-15), (center_x, center_y+15), color, 2)
            
            # Method bilgisi
            cv2.putText(display_frame, method, (center_x-20, center_y-25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # KAMERA MERKEZİ CROSSHAIR (HEDEF NİŞAN NOKTASI)
        camera_center_x = self.frame_width // 2
        camera_center_y = self.frame_height // 2
        
        # Ana merkez crosshair (yeşil)
        cv2.circle(display_frame, (camera_center_x, camera_center_y), 8, (0, 255, 0), 2)
        cv2.line(display_frame, (camera_center_x-20, camera_center_y), 
                 (camera_center_x+20, camera_center_y), (0, 255, 0), 2)
        cv2.line(display_frame, (camera_center_x, camera_center_y-20), 
                 (camera_center_x, camera_center_y+20), (0, 255, 0), 2)
        
        # Merkez etiketi
        cv2.putText(display_frame, "MERKEZ", (camera_center_x-25, camera_center_y-30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Eğer dart tespit edilmişse, merkez ile arasındaki mesafeyi göster
        if best_dart:
            if 'refined_center' in best_dart and best_dart['refined_center']:
                dart_center = best_dart['refined_center']
            else:
                x, y, w, h = best_dart['bbox']
                dart_center = (x + w//2, y + h//2)
            
            dart_x, dart_y = dart_center
            distance = math.sqrt((dart_x - camera_center_x)**2 + (dart_y - camera_center_y)**2)
            
            # Merkez ile dart arasındaki çizgi
            cv2.line(display_frame, (camera_center_x, camera_center_y), 
                     (dart_x, dart_y), (255, 255, 255), 1, cv2.LINE_AA)
            
            # Mesafe bilgisi
            cv2.putText(display_frame, f"Mesafe: {distance:.1f}px", 
                       (10, self.frame_height - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)        # Hedef kilitlenme progress
        if self.current_dart_target and self.targeting_start_time > 0:
            target_x, target_y = self.current_dart_target
            elapsed = current_time - self.targeting_start_time
            progress = min(elapsed / self.lock_duration, 1.0)
            
            # Progress bar
            bar_width = int(80 * progress)
            cv2.rectangle(display_frame, (target_x - 40, target_y + 30),
                         (target_x - 40 + bar_width, target_y + 35), (0, 255, 255), -1)
            cv2.rectangle(display_frame, (target_x - 40, target_y + 30),
                         (target_x + 40, target_y + 35), (255, 255, 255), 2)
            
            # Progress metni
            status = "LOCKED!" if progress >= 1.0 else f"LOCKING {progress*100:.0f}%"
            cv2.putText(display_frame, status, (target_x - 50, target_y + 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Sistem durumu
        current_pan = getattr(self.arduino_controller, 'current_pan', 90)
        current_tilt = getattr(self.arduino_controller, 'current_tilt', 90)
        laser_active = getattr(self.arduino_controller, 'laser_active', False)
        is_scanning = getattr(self.arduino_controller, 'is_scanning', False)
        target_locked = getattr(self.arduino_controller, 'target_locked', False)
        
        # Lazer durumu metni
        laser_status = "SYSTEM ON" if self.laser_enabled_by_system else ("MANUAL ON" if laser_active else "OFF")
        scan_status = "LOCKED" if target_locked else ("ON" if is_scanning else "OFF")
        
        arduino_info = [
            f"Arduino: Pan={current_pan:.1f}° Tilt={current_tilt:.1f}°",
            f"Laser: {laser_status} | Scan: {scan_status}",
            f"Threshold: {self.confidence_threshold:.2f}",
            f"Valid: {len(valid_detections)}/{len(detections)}",
            f"FPS: {fps:.1f} | Process: {process_time:.1f}ms"
        ]
        
        for i, info in enumerate(arduino_info):
            cv2.putText(display_frame, info, (10, 25 + i*20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Sistem durumu
        if self.laser_enabled_by_system:
            status_color = (0, 255, 0)  # Yeşil - sistem lazeri
            status_text = "LAZER AKTIF - HEDEF KİLİTLİ"
        elif laser_active:
            status_color = (0, 255, 255)  # Sarı - manuel lazer
            status_text = "MANUEL LAZER AKTIF"
        elif target_locked:
            status_color = (255, 255, 0)  # Mavi - hedef kilitli ama lazer kapalı
            status_text = "HEDEF KİLİTLİ - LAZER KAPALI"
        elif is_scanning:
            status_color = (255, 0, 255)  # Magenta - tarama aktif
            status_text = "TARAMA MODU AKTİF"
        else:
            status_color = (255, 255, 255)  # Beyaz - lazer kapalı
            status_text = "DART ARANIYOR - TARAMA KAPALI"
            
        cv2.putText(display_frame, status_text, (10, display_frame.shape[0] - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Frame merkezi (crosshair)
        if self.show_crosshair:
            center_x = self.frame_width // 2
            center_y = self.frame_height // 2
            cv2.circle(display_frame, (center_x, center_y), 3, (255, 0, 0), -1)
            cv2.line(display_frame, (center_x - 25, center_y), (center_x + 25, center_y), (255, 0, 0), 1)
            cv2.line(display_frame, (center_x, center_y - 25), (center_x, center_y + 25), (255, 0, 0), 1)
        
        # Kontrol bilgileri (sağ alt köşe)
        controls = [
            "Q/ESC:Quit", "Space:Laser", "C:Center", 
            "WASD:Move", "+/-:Threshold", "R:Reset",
            "F:ShowAll", "X:Crosshair", "T:Scan",
            "1/2/3:Speed", "Mouse:Aim"
        ]
        
        for i, control in enumerate(controls):
            y_pos = display_frame.shape[0] - 20 - (len(controls) - 1 - i) * 15
            cv2.putText(display_frame, control, (display_frame.shape[1] - 180, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        
        return display_frame


def main():
    """Ana program"""
    import argparse
    
    # Komut satırı argümanları
    parser = argparse.ArgumentParser(description='BARLAS YOLO+Arduino Dart Targeting System')
    parser.add_argument('--camera', type=int, default=0, help='Kamera index (0=dahili, 1=USB)')
    parser.add_argument('--arduino', type=str, default=None, help='Arduino port (örn: /dev/ttyACM0)')
    parser.add_argument('--list-cameras', action='store_true', help='Mevcut kameraları listele')
    args = parser.parse_args()
    
    # Kamera listesi isteniyorsa
    if args.list_cameras:
        print("🎥 Mevcut Kameralar:")
        for i in range(5):
            try:
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        h, w = frame.shape[:2]
                        print(f"  {i}: {'Dahili' if i==0 else 'USB'} Kamera - {w}x{h}")
                    cap.release()
            except:
                pass
        return
    
    print("🎯 BARLAS YOLO+ARDUINO DART LASER TARGETING SYSTEM")
    print("=" * 70)
    print("YOLO Dart Detection + Hough Circle + Arduino Pan-Tilt")
    print("Gelişmiş UI ve Kontrol Özellikleri")
    print("=" * 70)
    print(f"📹 Kamera: {args.camera} ({'Dahili' if args.camera==0 else 'USB'})")
    print(f"🔌 Arduino: {args.arduino if args.arduino else 'Otomatik tespit'}")
    print("=" * 70)
    
    try:
        # Sistem oluştur
        dart_system = YOLOArduinoDartSystem(camera_index=args.camera, arduino_port=args.arduino)
        
        # Sistemi başlat
        dart_system.run_dart_targeting()
        
    except Exception as e:
        print(f"❌ Sistem hatası: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
