#!/bin/bash

# BARLAS Dart Laser System - Linux Kurulum ve Test Scripti
# Ubuntu 20.04 / ROS Noetic için

echo "🎯 BARLAS Dart Laser System - Linux Kurulum Başlıyor 🎯"
echo "=" * 60

# Sistem güncellemesi
echo "📦 Sistem güncelleniyor..."
sudo apt update && sudo apt upgrade -y

# ROS Noetic kontrolü
if ! command -v roscore &> /dev/null; then
    echo "❌ ROS Noetic bulunamadı! ROS'u yükleyin:"
    echo "http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
else
    echo "✅ ROS Noetic mevcut"
fi

# Gerekli sistem paketleri
echo "📦 Sistem paketleri yükleniyor..."
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    git \
    v4l-utils \
    cheese

# Python paketleri
echo "🐍 Python paketleri yükleniyor..."
pip3 install --user \
    opencv-python>=4.8.0 \
    numpy>=1.21.0 \
    PyYAML>=6.0 \
    RPi.GPIO

# ROS paketleri
echo "🤖 ROS paketleri yükleniyor..."
sudo apt install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-usb-cam \
    ros-noetic-tf2-ros \
    ros-noetic-rqt \
    ros-noetic-rqt-image-view \
    ros-noetic-rviz

# Kamera testi
echo "📹 Kamera testi..."
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "✅ Kamera cihazları bulundu:"
    ls -la /dev/video*
    
    # V4L2 bilgileri
    for device in /dev/video*; do
        echo "Cihaz: $device"
        v4l2-ctl --device=$device --list-formats-ext 2>/dev/null | head -10
        echo "---"
    done
else
    echo "❌ Kamera cihazı bulunamadı!"
    echo "USB kamera bağlı mı kontrol edin"
fi

# GPIO test (Raspberry Pi için)
echo "🔌 GPIO test..."
if command -v pinout &> /dev/null; then
    echo "✅ Raspberry Pi GPIO mevcut"
    pinout
else
    echo "⚠️ Raspberry Pi GPIO bulunamadı - PC modunda çalışacak"
fi

# Servo bağlantı kontrolü
echo "🔧 Servo bağlantı kontrolü..."
echo "Servo motor bağlantı şeması:"
echo "Pan Servo  -> GPIO 18 (Pin 12)"
echo "Tilt Servo -> GPIO 19 (Pin 35)" 
echo "Lazer Diod -> GPIO 20 (Pin 38)"
echo "GND        -> Ground (Pin 6, 9, 14, 20, 25, 30, 34, 39)"
echo "5V         -> 5V Power (Pin 2, 4)"

# ROS workspace hazırlama
echo "🏗️ ROS workspace hazırlanıyor..."
if [ ! -d "$HOME/catkin_ws" ]; then
    mkdir -p $HOME/catkin_ws/src
    cd $HOME/catkin_ws
    catkin_make
    echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi

# BARLAS paketi kopyalama
echo "📁 BARLAS paketi hazırlanıyor..."
BARLAS_PKG_PATH="$HOME/catkin_ws/src/barlas_dart_laser"

if [ -d "$BARLAS_PKG_PATH" ]; then
    echo "⚠️ Paket zaten mevcut, güncellenecek..."
    rm -rf "$BARLAS_PKG_PATH"
fi

mkdir -p "$BARLAS_PKG_PATH"

# Bu script'in bulunduğu dizinden dosyaları kopyala
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cp -r "$SCRIPT_DIR/ros_nodes/"* "$BARLAS_PKG_PATH/"

# Python script'leri executable yap
chmod +x "$BARLAS_PKG_PATH/"*.py

# Catkin build
echo "🔨 ROS paketi build ediliyor..."
cd $HOME/catkin_ws
catkin_make

# Source setup
source $HOME/catkin_ws/devel/setup.bash

echo "✅ Kurulum tamamlandı!"
echo ""
echo "🚀 Test Başlatma Komutları:"
echo ""
echo "1. ROS Master başlat:"
echo "   roscore"
echo ""
echo "2. Servo Controller başlat (yeni terminal):"
echo "   rosrun barlas_dart_laser servo_controller_ros_node.py"
echo ""
echo "3. Dart Laser System başlat (yeni terminal):"
echo "   roslaunch barlas_dart_laser dart_laser_system.launch"
echo ""
echo "4. Test script çalıştır (yeni terminal):"
echo "   rosrun barlas_dart_laser ros_test_script.py"
echo ""
echo "🔧 Manuel Test Komutları:"
echo ""
echo "Servo merkez pozisyon:"
echo '   rostopic pub /barlas/servo/center_command std_msgs/Bool "data: true"'
echo ""
echo "Servo pozisyon ayarla:"
echo '   rostopic pub /barlas/servo/position_command std_msgs/Float32MultiArray "data: [90, 90]"'
echo ""
echo "Lazer aç/kapat:"
echo '   rostopic pub /barlas/servo/laser_command std_msgs/Bool "data: true"'
echo ""
echo "📊 İzleme Komutları:"
echo ""
echo "ROS topic'leri listele:"
echo "   rostopic list"
echo ""
echo "Servo durumu izle:"
echo "   rostopic echo /barlas/servo/current_position"
echo ""
echo "Dart tespit izle:"
echo "   rostopic echo /barlas/dart_laser/dart_detected"
echo ""
echo "Kamera görüntüsü izle:"
echo "   rqt_image_view"
echo ""

# Otomatik test başlatma seçeneği
read -p "Otomatik test başlatılsın mı? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🧪 Otomatik test başlatılıyor..."
    
    # ROS Master başlat (background)
    echo "ROS Master başlatılıyor..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
    
    # Servo Controller başlat (background)
    echo "Servo Controller başlatılıyor..."
    rosrun barlas_dart_laser servo_controller_ros_node.py &
    SERVO_PID=$!
    sleep 2
    
    # Test script çalıştır
    echo "Test script başlatılıyor..."
    rosrun barlas_dart_laser ros_test_script.py servo
    
    # Cleanup
    echo "Test tamamlandı, temizlik yapılıyor..."
    kill $SERVO_PID 2>/dev/null
    kill $ROSCORE_PID 2>/dev/null
    
else
    echo "Manuel test için yukarıdaki komutları kullanın"
fi

echo ""
echo "🎯 BARLAS Dart Laser System kurulum ve test tamamlandı! 🎯"
