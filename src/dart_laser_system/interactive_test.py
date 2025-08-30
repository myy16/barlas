#!/usr/bin/env python3
"""
BARLAS Interactive Targeting System Test
Kullanıcı dostu test menüsü
"""

import cv2
import time
import sys
from targeting_system import DartLaserTargetingSystem

def main_menu():
    """Ana test menüsü"""
    
    print("🎯 BARLAS DART LASER SYSTEM - INTERACTIVE TEST")
    print("=" * 55)
    print("📋 Test Seçenekleri:")
    print("1️⃣  Arduino Port Ayarla")
    print("2️⃣  YOLO Detection Test (Sadece Görüntü)")
    print("3️⃣  Pan-Tilt Servo Test")
    print("4️⃣  🔴 TAM SYSTEM - Otomatik Lazer Hedefleme")
    print("5️⃣  Sistem Bilgileri")
    print("0️⃣  Çıkış")
    print("=" * 55)
    
    arduino_port = "COM3"  # Varsayılan
    
    while True:
        try:
            print(f"\n📡 Mevcut Arduino Port: {arduino_port}")
            choice = input("🚀 Seçiminizi yapın (0-5): ").strip()
            
            if choice == "1":
                # Port ayarlama
                print("\n📡 Arduino Port Ayarlama:")
                print("Windows yaygın portları: COM1, COM2, COM3, COM4, COM5...")
                new_port = input(f"Yeni port (şu an: {arduino_port}): ").strip().upper()
                if new_port:
                    arduino_port = new_port
                    print(f"✅ Port güncellendi: {arduino_port}")
            
            elif choice == "2":
                # Sadece YOLO test
                test_yolo_only(arduino_port)
            
            elif choice == "3":
                # Pan-Tilt test
                test_pantilt(arduino_port)
            
            elif choice == "4":
                # Tam sistem
                test_full_system(arduino_port)
            
            elif choice == "5":
                # Sistem bilgileri
                show_system_info(arduino_port)
            
            elif choice == "0":
                print("👋 Görüşürüz!")
                break
            
            else:
                print("❌ Geçersiz seçim! Lütfen 0-5 arası bir rakam girin.")
        
        except KeyboardInterrupt:
            print("\n\n👋 Test sonlandırıldı (Ctrl+C)")
            break
        except Exception as e:
            print(f"❌ Beklenmeyen hata: {e}")

def test_yolo_only(port):
    """Sadece YOLO detection test eder"""
    print("\n📷 YOLO Detection Test")
    print("-" * 30)
    print("ℹ️  Bu test sadece dart algılamayı test eder")
    print("ℹ️  Arduino bağlantısı gerekmez")
    print("ℹ️  ESC tuşu ile çıkın")
    
    input("▶️  Başlatmak için Enter'e basın...")
    
    try:
        # Sadece detector oluştur
        system = DartLaserTargetingSystem(port)
        detector = system.dart_detector
        
        # Kamera başlat
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Kamera açılamadı!")
            return
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        frame_count = 0
        print("🎯 Detection başladı...")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_count += 1
            start_time = time.time()
            
            # YOLO + Hough detection
            detections = detector.detect_darts(frame)
            best_dart = detector.get_best_dart(detections)
            
            process_time = time.time() - start_time
            fps = 1.0 / process_time if process_time > 0 else 0
            
            # Görselleştirme
            display_frame = frame.copy()
            
            if best_dart:
                x, y, w, h = best_dart['bbox']
                center_x, center_y = best_dart['center']
                
                # Dart kutusunu çiz
                cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(display_frame, (center_x, center_y), 8, (0, 0, 255), -1)
                
                # Bilgi metni
                conf = best_dart['confidence']
                info_text = f"Conf: {conf:.2f}"
                if best_dart.get('refined', False):
                    radius = best_dart.get('hough_radius', '?')
                    info_text += f" [Hough R={radius}]"
                
                cv2.putText(display_frame, info_text, (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                print(f"🎯 Frame {frame_count}: Dart merkezi ({center_x}, {center_y}) - {info_text}")
            
            else:
                print(f"🔍 Frame {frame_count}: Dart algılanamadı")
            
            # FPS bilgisi
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, f"Frame: {frame_count}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('YOLO Detection Test', display_frame)
            
            # ESC ile çık
            if cv2.waitKey(1) & 0xFF == 27:
                break
        
        cap.release()
        cv2.destroyAllWindows()
        print(f"✅ Test tamamlandı - {frame_count} frame işlendi")
    
    except Exception as e:
        print(f"❌ YOLO test hatası: {e}")

def test_pantilt(port):
    """Pan-Tilt servo test eder"""
    print(f"\n🧭 Pan-Tilt Test ({port})")
    print("-" * 30)
    
    try:
        system = DartLaserTargetingSystem(port)
        pantilt = system.laser_pantilt
        
        print("✅ Pan-Tilt bağlantısı kuruldu")
        print("📋 Test sırası:")
        
        # Test sırası
        print("1. Merkeze dön...")
        pantilt.center()
        time.sleep(2)
        
        print("2. Pan (yatay) test - Sol/Sağ...")
        for angle in [-30, -15, 0, 15, 30, 0]:
            print(f"   Pan açısı: {angle}°")
            pantilt.pan(angle)
            time.sleep(1)
        
        print("3. Tilt (dikey) test - Yukarı/Aşağı...")
        for angle in [-20, -10, 0, 10, 20, 0]:
            print(f"   Tilt açısı: {angle}°")
            pantilt.tilt(angle)
            time.sleep(1)
        
        print("4. Kombine hareket testi...")
        movements = [(-20, -15), (20, 15), (0, 0)]
        for pan_angle, tilt_angle in movements:
            print(f"   Pan: {pan_angle}°, Tilt: {tilt_angle}°")
            pantilt.move(pan_angle, tilt_angle)
            time.sleep(1.5)
        
        print("5. Merkeze geri dön...")
        pantilt.center()
        
        print("✅ Pan-Tilt test tamamlandı!")
    
    except Exception as e:
        print(f"❌ Pan-Tilt test hatası: {e}")
        print("💡 Kontrol edin: Arduino bağlantısı, port, servo kablolar")

def test_full_system(port):
    """Tam otomatik sistem test eder"""
    print(f"\n🚀 TAM SİSTEM TEST ({port})")
    print("⚠️" * 20)
    print("🔴 BU TEST OTOMATIK LAZER ATEŞLEYECEKTİR!")
    print("⚠️  Lazer güvenlik önlemlerini aldığınızdan emin olun")
    print("⚠️  Sadece güvenli test ortamında kullanın")
    print("⚠️" * 20)
    
    confirm = input("🤔 Devam etmek istediğinizden EMİN misiniz? (EVET/hayır): ")
    if confirm.upper() != "EVET":
        print("❌ Test iptal edildi")
        return
    
    try:
        print("🎯 Tam sistem başlatılıyor...")
        system = DartLaserTargetingSystem(port)
        
        print("✅ Sistem hazır!")
        print("📋 Otomatik işlemler:")
        print("  🎯 YOLO dart algılama")
        print("  🔍 Hough Circle merkez düzeltmesi")
        print("  🧭 Pan-Tilt otomatik hedefleme")
        print("  🔴 2 saniye kilit sonrası lazer atışı")
        print("  ⏱️  ESC tuşu ile acil durdurma")
        
        input("▶️  Başlatmak için Enter'e basın...")
        
        # Targeting başlat
        system.start_targeting()
        print("🔥 OTOMATİK HEDEFLEME BAŞLADI!")
        
        # ESC ile durdurma kontrolü
        while True:
            if cv2.waitKey(100) & 0xFF == 27:  # ESC
                print("⏹️  ESC tuşu - Sistem durduruluyor...")
                break
            time.sleep(0.1)
        
        system.stop_targeting()
        cv2.destroyAllWindows()
        print("✅ Tam sistem test tamamlandı")
    
    except Exception as e:
        print(f"❌ Tam sistem test hatası: {e}")

def show_system_info(port):
    """Sistem bilgilerini gösterir"""
    print(f"\n📊 SİSTEM BİLGİLERİ")
    print("-" * 40)
    print(f"📡 Arduino Port: {port}")
    print(f"🎯 YOLO Model: dart_recognize/best.onnx")
    print(f"🔍 Hough Circle: Aktif")
    print(f"🧭 Pan-Tilt: Arduino Servo")
    print(f"🔴 Lazer: PWM kontrol")
    print(f"📷 Kamera: USB (varsayılan: 0)")
    print(f"⚙️  İşlem: Windows PowerShell")
    
    # Test bağlantısı
    try:
        print(f"\n🔌 Bağlantı testi ({port})...")
        system = DartLaserTargetingSystem(port)
        print("✅ Arduino bağlantısı başarılı")
        
        # Detector test
        detector = system.dart_detector
        print("✅ YOLO detector hazır")
        
        # Kamera test
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            print("✅ Kamera erişilebilir")
            cap.release()
        else:
            print("❌ Kamera problemi")
        
    except Exception as e:
        print(f"❌ Bağlantı hatası: {e}")

if __name__ == "__main__":
    main_menu()
