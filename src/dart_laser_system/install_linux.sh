#!/bin/bash

# BARLAS Arduino Dart System - Linux/Mac Kurulum Script
# Ubuntu 20.04+ / macOS 10.15+ için

echo "🎯 BARLAS Arduino Dart System Kurulumu Başlıyor..."
echo "=" * 60

# 1. Python3 Kontrol
echo "🐍 Python3 kontrol ediliyor..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1)
    echo "✅ $PYTHON_VERSION mevcut"
else
    echo "❌ Python3 bulunamadı!"
    echo "Ubuntu: sudo apt install python3 python3-pip"
    echo "macOS: brew install python3"
    exit 1
fi

# 2. Pip3 Kontrol
echo "📦 Pip3 kontrol ediliyor..."
if command -v pip3 &> /dev/null; then
    echo "✅ Pip3 mevcut"
    pip3 install --upgrade pip
else
    echo "❌ Pip3 bulunamadı!"
    echo "Ubuntu: sudo apt install python3-pip"
    exit 1
fi

# 3. Sistem Paketleri (Ubuntu için)
if command -v apt &> /dev/null; then
    echo "📦 Ubuntu sistem paketleri yükleniyor..."
    sudo apt update
    sudo apt install -y \
        python3-dev \
        python3-opencv \
        libopencv-dev \
        v4l-utils \
        gcc \
        g++
fi

# 4. Python Kütüphaneleri
echo "📚 Python kütüphaneleri yükleniyor..."

PACKAGES=(
    "opencv-python>=4.8.0"
    "numpy>=1.21.0" 
    "PyYAML>=6.0"
    "pyserial>=3.5"
)

for package in "${PACKAGES[@]}"; do
    echo "  Yükleniyor: $package"
    pip3 install "$package"
    
    if [ $? -eq 0 ]; then
        echo "  ✅ $package - OK"
    else
        echo "  ❌ $package - HATA"
    fi
done

# 5. Opsiyonel Kütüphaneler
echo "🎨 Opsiyonel kütüphaneler yükleniyor..."

OPTIONAL_PACKAGES=(
    "onnxruntime"
    "matplotlib" 
    "pillow"
)

for package in "${OPTIONAL_PACKAGES[@]}"; do
    echo "  Yükleniyor: $package"
    pip3 install "$package" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo "  ✅ $package - OK"
    else
        echo "  ⚠️ $package - Atlandı (opsiyonel)"
    fi
done

# 6. Import Testleri
echo "🧪 Import testleri yapılıyor..."

IMPORTS=("cv2:OpenCV" "numpy:NumPy" "serial:PySerial" "yaml:PyYAML")

for import_test in "${IMPORTS[@]}"; do
    module="${import_test%:*}"
    name="${import_test#*:}"
    
    if python3 -c "import $module" 2>/dev/null; then
        echo "  ✅ $name - OK"
    else
        echo "  ❌ $name - HATA"
    fi
done

# 7. Kamera Kontrolü
echo "📹 Kamera kontrolü yapılıyor..."

if ls /dev/video* 1> /dev/null 2>&1; then
    echo "  ✅ Video cihazları bulundu:"
    ls -la /dev/video* | while read line; do
        echo "    $line"
    done
    
    # V4L2 bilgileri (varsa)
    if command -v v4l2-ctl &> /dev/null; then
        echo "  📊 Kamera detayları:"
        for device in /dev/video*; do
            echo "    $device:"
            v4l2-ctl --device=$device --list-formats-ext 2>/dev/null | head -5
        done
    fi
else
    echo "  ⚠️ Video cihazı bulunamadı"
fi

# 8. Arduino USB Kontrolü
echo "🤖 Arduino USB kontrolü yapılıyor..."

USB_DEVICES=$(lsusb | grep -i "arduino\|ch340\|ftdi\|cp210")

if [ ! -z "$USB_DEVICES" ]; then
    echo "  ✅ Arduino benzeri cihazlar bulundu:"
    echo "$USB_DEVICES" | while read line; do
        echo "    $line"
    done
else
    echo "  ⚠️ Arduino cihazı bulunamadı (USB bağlı değil)"
fi

# Serial portları kontrol et
if ls /dev/ttyUSB* /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "  ✅ Serial portlar:"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | while read line; do
        echo "    $line"
    done
else
    echo "  ⚠️ Serial port bulunamadı"
fi

# 9. İzin Kontrolü
echo "🔐 Kullanıcı izinleri kontrol ediliyor..."

GROUPS_TO_CHECK=("dialout" "tty" "video")

for group in "${GROUPS_TO_CHECK[@]}"; do
    if groups | grep -q "\b$group\b"; then
        echo "  ✅ $group grubu - OK"
    else
        echo "  ⚠️ $group grubu - EKSIK"
        echo "    Çözüm: sudo usermod -a -G $group \$USER"
        echo "    Sonra çıkış yapıp tekrar giriş yapın"
    fi
done

# 10. Sistem Dosyaları Kontrolü
echo "📂 Sistem dosyaları kontrol ediliyor..."

REQUIRED_FILES=(
    "arduino_controller.py"
    "arduino_pantilt.ino" 
    "yolo_arduino_dart_system.py"
    "camera_test.py"
    "requirements.txt"
)

MISSING_FILES=()

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✅ $file - OK"
    else
        echo "  ❌ $file - EKSIK"
        MISSING_FILES+=("$file")
    fi
done

# 11. Kurulum Özeti
echo ""
echo "📋 KURULUM ÖZETİ"
echo "=" * 60

# Sistem durumu kontrol
SYSTEM_READY=true

# Python kontrol
if python3 -c "import cv2, numpy, serial, yaml" 2>/dev/null; then
    echo "✅ Python Kütüphaneleri: HAZIR"
else
    echo "❌ Python Kütüphaneleri: EKSIK"
    SYSTEM_READY=false
fi

# YOLO kontrol
if [ -f "../dart_recognize/yolo_predictions.py" ]; then
    echo "✅ YOLO Modülü: MEVCUT"
else
    echo "⚠️ YOLO Modülü: KONTROL EDİN"
fi

# Dosya kontrolü
if [ ${#MISSING_FILES[@]} -eq 0 ]; then
    echo "✅ Sistem Dosyaları: TAMAM"
else
    echo "❌ Sistem Dosyaları: EKSIK (${MISSING_FILES[*]})"
    SYSTEM_READY=false
fi

echo ""

if [ "$SYSTEM_READY" = true ]; then
    echo "🎉 SİSTEM HAZIR!"
    echo ""
    echo "🚀 Başlatma Komutları:"
    echo "  cd src/dart_laser_system"
    echo "  python3 yolo_arduino_dart_system.py"
    echo ""
    echo "📋 Test Komutları:"  
    echo "  python3 camera_test.py"
    echo "  python3 yolo_arduino_dart_system.py --list-cameras"
else
    echo "⚠️ SİSTEM HAZIR DEĞİL!"
    echo "Lütfen eksik kütüphaneleri ve dosyaları kontrol edin"
fi

echo ""
echo "📖 Detaylı bilgi için INSTALLATION_GUIDE.md dosyasını okuyun"
echo "🎯 BARLAS Arduino Dart Targeting System"
