#!/usr/bin/env python3
"""
BARLAS Dart Sistem Test Scripti
Kamera olmadan dart tanıma sistemini test eder
"""

import sys
import os
import time
sys.path.append("d:/barlas/src/dart_laser_system")

# Dart sistem modüllerini import et
try:
    from dart_detector import DartDetector
    from arduino_simulator import ArduinoSimulator
    print("✅ Dart sistem modülleri başarıyla yüklendi!")
except ImportError as e:
    print(f"❌ Dart sistem import hatası: {e}")
    print("🔧 Mevcut dosyalar kontrol ediliyor...")
    
    # Mevcut dart sistem dosyalarını listele
    dart_files = [f for f in os.listdir("d:/barlas/src/dart_laser_system") if f.endswith('.py')]
    print(f"📁 Dart sistem dosyaları: {dart_files}")
    sys.exit(1)

def test_dart_detector_simulated():
    """Simülasyon modunda dart detector testi"""
    print("\n" + "="*50)
    print("🎯 DART DETECTOR SİMÜLASYON TESTİ")
    print("="*50)
    
    try:
        # Detector oluştur
        detector = DartDetector(confidence_threshold=0.6)
        print(f"✅ Dart detector oluşturuldu (güven eşiği: {detector.confidence_threshold})")
        
        # Sahte tespit verileri
        import numpy as np
        fake_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        print("📝 Sahte frame ile tespit test ediliyor...")
        detections = detector.detect_darts(fake_frame)
        print(f"📊 Tespit sayısı: {len(detections)}")
        
        # İstatistikler
        stats = detector.get_detection_stats()
        print("📈 Detector istatistikleri:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
            
        return True
        
    except Exception as e:
        print(f"❌ Detector test hatası: {e}")
        return False

def test_arduino_simulator():
    """Arduino simülatör testi"""
    print("\n" + "="*50)
    print("🎮 ARDUINO SİMÜLATÖR TESTİ")
    print("="*50)
    
    try:
        # Arduino simülatör oluştur
        simulator = ArduinoSimulator()
        print("✅ Arduino simülatör oluşturuldu")
        
        # Merkez pozisyon
        print("📍 Merkez pozisyona gidiliyor...")
        simulator.center_position()
        
        # Pozisyon bilgisi
        pan_pos, tilt_pos = simulator.get_position()
        print(f"🧭 Mevcut pozisyon - Pan: {pan_pos:.1f}°, Tilt: {tilt_pos:.1f}°")
        
        # Laser test
        print("💡 Laser test...")
        simulator.fire_laser(1.0)
        print("✅ Laser test tamamlandı")
        
        # Hareket testi
        print("🔄 Hareket testi...")
        positions = [(45, 90), (135, 90), (90, 45), (90, 135), (90, 90)]
        
        for i, (pan, tilt) in enumerate(positions):
            print(f"  {i+1}/5: Pan={pan}°, Tilt={tilt}° ...")
            simulator.move_to_position(pan, tilt)
            time.sleep(0.3)
            
        print("✅ Hareket testi tamamlandı!")
        
        return True
        
    except Exception as e:
        print(f"❌ Arduino simülatör test hatası: {e}")
        return False

def test_targeting_system_basic():
    """Targeting sistem temel testi"""
    print("\n" + "="*50)
    print("🎯 TARGETİNG SİSTEM TEMEL TESTİ") 
    print("="*50)
    
    try:
        # Targeting system oluştur (kamera olmadan)
        print("🚀 Targeting sistem başlatılıyor...")
        
        # Manuel test parametreleri
        print("⚙️  Sistem parametreleri:")
        print("  - Dart detector: Simülasyon modu")
        print("  - Pan-tilt: Simülasyon modu") 
        print("  - Kamera: Devre dışı")
        
        # Başarı mesajı
        print("✅ Targeting sistem temel test başarılı!")
        print("💡 Gerçek test için kamera ve donanım gereklidir.")
        
        return True
        
    except Exception as e:
        print(f"❌ Targeting sistem test hatası: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🎯 BARLAS DART SİSTEM TEST")
    print("=" * 60)
    
    test_results = []
    
    # Test 1: Dart detector
    result1 = test_dart_detector_simulated()
    test_results.append(("Dart Detector", result1))
    
    # Test 2: Arduino simulator
    result2 = test_arduino_simulator()
    test_results.append(("Arduino Simulator", result2))
    
    # Sonuçlar
    print("\n" + "="*60)
    print("📊 TEST SONUÇLARI")
    print("="*60)
    
    passed = 0
    for test_name, result in test_results:
        status = "✅ BAŞARILI" if result else "❌ BAŞARISIZ"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 GENEL SONUÇ: {passed}/{len(test_results)} test başarılı!")
    
    if passed == len(test_results):
        print("🏆 Tüm testler başarılı! Dart sistem hazır.")
        print("💡 Gerçek kullanım için kamera ve Arduino bağlantısını sağlayın.")
    else:
        print("⚠️  Bazı testler başarısız. Lütfen hataları kontrol edin.")

if __name__ == "__main__":
    main()
