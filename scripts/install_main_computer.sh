#!/bin/bash
# BARLAS Ana Bilgisayar Kurulum Scripti
# MSI Laptop i5-13420H + RTX 4060 İçin Hibrit Sistem
# WSL2 Ubuntu + ROS Noetic + Mission Planner + CUDA

set -e

echo "🎯 BARLAS ANA BİLGİSAYAR KURULUM BAŞLATIYOR..."
echo "Sistem: MSI Laptop i5-13420H + RTX 4060"
echo "Hedef: WSL2 + ROS Noetic + Mission Planner Hibrit Sistemi"
echo "==========================================================="

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

# Hata yakalama fonksiyonu
error_exit() {
    echo -e "${RED}❌ HATA: $1${NC}" >&2
    exit 1
}

# İlerleme göstergesi
progress() {
    echo -e "${BLUE}🔄 $1...${NC}"
}

# Başarı mesajı
success() {
    echo -e "${GREEN}✅ $1${NC}"
}

# Uyarı mesajı
warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

# Sistem bilgilerini kontrol et
echo -e "${PURPLE}📊 Sistem Bilgileri Kontrol Ediliyor...${NC}"
echo "Hostname: $(hostname)"
echo "Kernel: $(uname -r)"
echo "CPU: $(grep 'model name' /proc/cpuinfo | head -1 | cut -d':' -f2)"
echo "Memory: $(free -h | grep '^Mem:' | awk '{print $2}')"
echo "WSL Version: $(cat /proc/version | grep Microsoft)"

# === ADIM 1: Temel Sistem Güncellemeleri ===
progress "Sistem paketleri güncelleniyor"
sudo apt update && sudo apt upgrade -y || error_exit "Sistem güncellemesi başarısız"
success "Sistem güncellemeleri tamamlandı"

# === ADIM 2: ROS Noetic Kurulumu ===
progress "ROS Noetic kurulum kontrol ediliyor"

if ! command -v roscore &> /dev/null; then
    progress "ROS Noetic kuruluyor..."
    
    # ROS repository ekleme
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    
    # ROS Noetic full kurulumu
    sudo apt install ros-noetic-desktop-full -y || error_exit "ROS kurulumu başarısız"
    
    # rosdep kurulum
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    sudo rosdep init || warning "rosdep zaten initialize edilmiş"
    rosdep update
    
    # ROS environment setup
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source /opt/ros/noetic/setup.bash
    
    success "ROS Noetic kurulumu tamamlandı"
else
    success "ROS Noetic zaten kurulu"
fi

# === ADIM 3: MAVROS ve ArduPilot Bağımlılıkları ===
progress "MAVROS ve ArduPilot bileşenleri kuruluyor"

sudo apt install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-geographic-msgs \
    python3-pip \
    git \
    wget \
    || error_exit "MAVROS kurulumu başarısız"

# GeographicLib datasets
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh || warning "GeographicLib zaten kurulu"

success "MAVROS kurulumu tamamlandı"

# === ADIM 4: CUDA ve GPU Desteği ===
progress "NVIDIA CUDA desteği kontrol ediliyor"

if nvidia-smi &> /dev/null; then
    success "NVIDIA GPU tespit edildi"
    
    # CUDA kurulumu WSL2 için
    if ! command -v nvcc &> /dev/null; then
        progress "CUDA Toolkit kuruluyor..."
        
        # NVIDIA CUDA WSL2 repository
        wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.0-1_all.deb
        sudo dpkg -i cuda-keyring_1.0-1_all.deb
        sudo apt update
        sudo apt install cuda-toolkit-12-2 -y || error_exit "CUDA kurulumu başarısız"
        
        # CUDA PATH
        echo 'export PATH=/usr/local/cuda-12.2/bin${PATH:+:${PATH}}' >> ~/.bashrc
        echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
        
        success "CUDA kurulumu tamamlandı"
    else
        success "CUDA zaten kurulu"
    fi
else
    warning "NVIDIA GPU bulunamadı - CPU modunda çalışacak"
fi

# === ADIM 5: Python ML Kütüphaneleri ===
progress "Python ML kütüphaneleri kuruluyor"

pip3 install --user \
    torch torchvision \
    ultralytics \
    opencv-python \
    numpy \
    pyserial \
    pymavlink \
    dronekit \
    matplotlib \
    scipy \
    || error_exit "Python kütüphane kurulumu başarısız"

success "Python ML kütüphaneleri kuruldu"

# === ADIM 6: ArduPilot SITL Kurulumu ===
progress "ArduPilot SITL kuruluyor"

ARDUPILOT_DIR="$HOME/ardupilot"

if [ ! -d "$ARDUPILOT_DIR" ]; then
    progress "ArduPilot repository klonlanıyor..."
    cd $HOME
    git clone https://github.com/ArduPilot/ardupilot.git || error_exit "ArduPilot klonlama başarısız"
    cd ardupilot
    git submodule update --init --recursive || error_exit "Submodule güncellemesi başarısız"
    
    # Build araçları kurulumu
    Tools/environment_install/install-prereqs-ubuntu.sh -y || error_exit "ArduPilot önkoşul kurulumu başarısız"
    
    # SITL build
    ./waf configure --board sitl || error_exit "ArduPilot configure başarısız"
    ./waf rover || error_exit "ArduPilot Rover build başarısız"
    
    success "ArduPilot SITL kurulumu tamamlandı"
else
    success "ArduPilot zaten kurulu"
fi

# === ADIM 7: Gazebo Simülasyon Ortamı ===
progress "Gazebo simülasyon kurulumu kontrol ediliyor"

if ! command -v gazebo &> /dev/null; then
    progress "Gazebo kuruluyor..."
    
    sudo apt install -y \
        gazebo11 \
        libgazebo11-dev \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        || error_exit "Gazebo kurulumu başarısız"
        
    success "Gazebo kurulumu tamamlandı"
else
    success "Gazebo zaten kurulu"
fi

# ArduPilot Gazebo Plugin
GAZEBO_PLUGIN_DIR="$HOME/ardupilot_gazebo"

if [ ! -d "$GAZEBO_PLUGIN_DIR" ]; then
    progress "ArduPilot Gazebo plugin kuruluyor..."
    cd $HOME
    git clone https://github.com/khancyr/ardupilot_gazebo || error_exit "Gazebo plugin klonlama başarısız"
    cd ardupilot_gazebo
    mkdir build && cd build
    cmake .. && make -j4 || error_exit "Gazebo plugin build başarısız"
    
    # Environment variable
    echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:$GAZEBO_PLUGIN_DIR/build" >> ~/.bashrc
    
    success "ArduPilot Gazebo plugin kurulumu tamamlandı"
else
    success "ArduPilot Gazebo plugin zaten kurulu"
fi

# === ADIM 8: BARLAS Workspace Kurulumu ===
progress "BARLAS ROS workspace kuruluyor"

WORKSPACE_DIR="$HOME/barlas_ws"

if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p $WORKSPACE_DIR/src
    cd $WORKSPACE_DIR
    catkin_make || error_exit "Workspace build başarısız"
    
    # Workspace environment
    echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
    
    success "BARLAS workspace oluşturuldu"
else
    success "BARLAS workspace zaten mevcut"
fi

# === ADIM 9: Network Konfigürasyonu ===
progress "WSL2 network konfigürasyonu yapılıyor"

# WSL IP'yi al
WSL_IP=$(hostname -I | awk '{print $1}')

# ROS network ayarları
echo "export ROS_IP=$WSL_IP" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://\$ROS_IP:11311" >> ~/.bashrc

success "Network konfigürasyonu tamamlandı"

# === ADIM 10: BARLAS Paket Kopyalama ===
progress "BARLAS paketleri workspace'e kopyalanıyor"

# Bu script'in çalıştığı dizini bul
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BARLAS_SOURCE_DIR="$(dirname "$SCRIPT_DIR")"

# BARLAS ana paketi kopyala
cp -r "$BARLAS_SOURCE_DIR" "$WORKSPACE_DIR/src/" || error_exit "BARLAS paket kopyalama başarısız"

# Workspace build et
cd $WORKSPACE_DIR
catkin_make || error_exit "BARLAS paket build başarısız"

success "BARLAS paketleri kuruldu"

# === FINAL: Kurulum Özeti ===
echo ""
echo -e "${GREEN}🎉 BARLAS ANA BİLGİSAYAR KURULUMU TAMAMLANDI! 🎉${NC}"
echo "==========================================================="
echo ""
echo -e "${BLUE}📋 Kurulan Bileşenler:${NC}"
echo "✅ ROS Noetic Desktop Full"
echo "✅ MAVROS + GeographicLib"
echo "✅ NVIDIA CUDA Toolkit (GPU varsa)"
echo "✅ Python ML Stack (PyTorch, YOLO, OpenCV)"
echo "✅ ArduPilot SITL Rover"
echo "✅ Gazebo 11 + ArduPilot Plugin"
echo "✅ BARLAS ROS Workspace"
echo ""
echo -e "${YELLOW}🚀 Sistem Başlatma Komutları:${NC}"
echo ""
echo "# Terminal 1 - ArduPilot SITL:"
echo "cd ~/ardupilot"
echo "python3 Tools/autotest/sim_vehicle.py -v Rover --console --out=udp:0.0.0.0:14550"
echo ""
echo "# Terminal 2 - BARLAS Hibrit System:"
echo "source ~/barlas_ws/devel/setup.bash"
echo "roslaunch barlas barlas_hibrid_system.launch"
echo ""
echo "# Windows Mission Planner:"
echo "UDP Connection: localhost:14550"
echo ""
echo -e "${PURPLE}🔧 Network Ayarları (Windows PowerShell Admin):${NC}"
echo "netsh interface portproxy add v4tov4 listenport=5760 listenaddress=0.0.0.0 connectport=5760 connectaddress=$WSL_IP"
echo "netsh interface portproxy add v4tov4 listenport=14550 listenaddress=0.0.0.0 connectport=14550 connectaddress=$WSL_IP"
echo ""
echo -e "${GREEN}✨ BARLAS hibrit sistemi kullanıma hazır!${NC}"
echo ""

# Yeniden başlatma önerisi
warning "Kurulumun tam aktif olması için WSL2'yi yeniden başlatın:"
warning "Windows PowerShell: wsl --shutdown && wsl"
