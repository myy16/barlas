#!/usr/bin/env python3
"""
🚀 BARLAS Sistem Kontrol Paneli
Arkadaşınızın sistem durumunu hızlıca kontrol edebilmesi için
"""

import sys
import os
import subprocess

def print_header():
    print("=" * 60)
    print("🎯 BARLAS SİSTEM KONTROL PANELİ")
    print("=" * 60)

def check_python_packages():
    """Gerekli Python paketlerini kontrol et"""
    print("\n📦 Python Paket Kontrolü:")
    
    required_packages = [
        'cv2', 'serial', 'numpy', 'ultralytics', 'PIL'
    ]
    
    missing_packages = []
    
    for package in required_packages:
        try:
            if package == 'cv2':
                import cv2
                print(f"  ✅ OpenCV: {cv2.__version__}")
            elif package == 'serial':
                import serial
                print(f"  ✅ PySerial: {serial.__version__}")
            elif package == 'numpy':
                import numpy as np
                print(f"  ✅ NumPy: {np.__version__}")
            elif package == 'ultralytics':
                import ultralytics
                print(f"  ✅ Ultralytics: OK")
            elif package == 'PIL':
                import PIL
                print(f"  ✅ Pillow: {PIL.__version__}")
        except ImportError:
            missing_packages.append(package)
            package_name = {
                'cv2': 'opencv-python',
                'serial': 'pyserial', 
                'PIL': 'Pillow'
            }.get(package, package)
            print(f"  ❌ {package_name}: EKSIK")
    
    return len(missing_packages) == 0

def check_arduino_ports():
    """Mevcut Arduino portlarını listele"""
    print("\n🔌 Arduino Port Kontrolü:")
    
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        if ports:
            for port in ports:
                print(f"  📍 Port: {port.device}")
                print(f"     Açıklama: {port.description}")
                if "Arduino" in port.description or "USB" in port.description:
                    print(f"     ✅ Muhtemel Arduino portu")
                print()
        else:
            print("  ❌ Hiç port bulunamadı!")
            
    except ImportError:
        print("  ❌ pyserial kurulu değil!")
        
    return len(ports) > 0 if 'ports' in locals() else False

def check_camera():
    """Kamera erişimini kontrol et"""
    print("\n📹 Kamera Kontrolü:")
    
    try:
        import cv2
        
        for i in range(3):  # 0, 1, 2 indekslerini dene
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"  ✅ Kamera {i}: Çalışıyor ({frame.shape[1]}x{frame.shape[0]})")
                else:
                    print(f"  ⚠️  Kamera {i}: Açık ama görüntü yok")
                cap.release()
            else:
                print(f"  ❌ Kamera {i}: Erişilemez")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Kamera hatası: {e}")
        return False

def check_files():
    """Gerekli dosyaları kontrol et"""
    print("\n📁 Dosya Kontrolü:")
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    required_files = [
        'arduino_pantilt_fixed.ino',
        'yolo_arduino_dart_system.py',
        'arduino_controller_fixed.py',
        'test_system_integration.py'
    ]
    
    optional_files = [
        'best.onnx'
    ]
    
    all_good = True
    
    for file in required_files:
        file_path = os.path.join(current_dir, file)
        if os.path.exists(file_path):
            size = os.path.getsize(file_path)
            print(f"  ✅ {file}: OK ({size} bytes)")
        else:
            print(f"  ❌ {file}: EKSIK!")
            all_good = False
    
    for file in optional_files:
        file_path = os.path.join(current_dir, file)
        if os.path.exists(file_path):
            size = os.path.getsize(file_path)
            print(f"  ✅ {file}: OK ({size} bytes)")
        else:
            print(f"  ⚠️  {file}: EKSIK (YOLO model dosyası)")
    
    return all_good

def run_quick_test():
    """Hızlı Arduino testi"""
    print("\n🧪 Hızlı Arduino Testi:")
    
    try:
        # Test scriptini çalıştır
        result = subprocess.run([sys.executable, 'test_system_integration.py'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("  ✅ Test scripti çalıştı")
        else:
            print("  ❌ Test scripti hatası")
            print(f"     Hata: {result.stderr}")
            
    except Exception as e:
        print(f"  ❌ Test hatası: {e}")

def main():
    print_header()
    
    # Tüm kontrolleri yap
    packages_ok = check_python_packages()
    ports_ok = check_arduino_ports() 
    camera_ok = check_camera()
    files_ok = check_files()
    
    # Özet
    print("\n" + "="*60)
    print("📋 ÖZET:")
    
    if packages_ok and ports_ok and camera_ok and files_ok:
        print("🎉 TÜM SİSTEMLER HAZIR!")
        print("✅ Sistem çalıştırmaya hazır")
        print("\n💡 Şimdi yapabileceklerin:")
        print("   python test_system_integration.py   # Test")
        print("   python yolo_arduino_dart_system.py  # Ana sistem")
        
    else:
        print("⚠️  BAZI SORUNLAR VAR:")
        if not packages_ok:
            print("   📦 Eksik Python paketleri var")
        if not ports_ok:
            print("   🔌 Arduino port problemi")
        if not camera_ok:
            print("   📹 Kamera problemi")
        if not files_ok:
            print("   📁 Eksik dosyalar var")
            
        print("\n🔧 ÇÖZÜM:")
        print("   KURULUM_REHBERİ.md dosyasını oku")
    
    print("="*60)

if __name__ == "__main__":
    main()
