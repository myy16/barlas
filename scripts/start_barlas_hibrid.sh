#!/bin/bash
# BARLAS Hibrit Sistem Başlatma Script'i
# WSL2 Ubuntu + ROS Noetic + Mission Planner Integration

set -e  # Exit on error

echo "🎯 BARLAS Hibrit Sistem Başlatılıyor..."
echo "WSL2 + ROS Noetic + Mission Planner + Dart Laser System"
echo "=================================================="

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ROS Environment kontrol
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS environment yüklü değil!${NC}"
    echo "ROS source edilmeli:"
    echo "source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo -e "${GREEN}✅ ROS $ROS_DISTRO environment aktif${NC}"

# MAVROS kontrolü
if ! rospack find mavros > /dev/null 2>&1; then
    echo -e "${RED}❌ MAVROS yüklü değil!${NC}"
    echo "MAVROS kurulumu için:"
    echo "sudo apt install ros-noetic-mavros ros-noetic-mavros-extras"
    exit 1
fi

echo -e "${GREEN}✅ MAVROS hazır${NC}"

# Workspace kontrolü
if [ ! -d "$HOME/barlas_ws" ]; then
    echo -e "${YELLOW}⚠️  BARLAS workspace bulunamadı, oluşturuluyor...${NC}"
    mkdir -p $HOME/barlas_ws/src
    cd $HOME/barlas_ws
    catkin_make
    echo "source $HOME/barlas_ws/devel/setup.bash" >> ~/.bashrc
fi

# Workspace source et
source $HOME/barlas_ws/devel/setup.bash
echo -e "${GREEN}✅ BARLAS workspace hazır${NC}"

# Network ayarları
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://$ROS_IP:11311
echo -e "${BLUE}🌐 ROS Network: $ROS_IP:11311${NC}"

# Port forwarding kontrolü (Windows PowerShell gerekiyor)
echo -e "${YELLOW}📡 Network port forwarding kontrol ediliyor...${NC}"
# Windows'ta çalıştırılacak komutlar için not
echo "Windows PowerShell'de bu komutları çalıştırın:"
echo "netsh interface portproxy add v4tov4 listenport=5760 listenaddress=0.0.0.0 connectport=5760 connectaddress=$ROS_IP"
echo "netsh interface portproxy add v4tov4 listenport=14550 listenaddress=0.0.0.0 connectport=14550 connectaddress=$ROS_IP"

# Başlatma seçenekleri
echo ""
echo "🚀 BAŞLATMA SEÇENEKLERİ:"
echo "1) Tam sistem (ArduPilot SITL + MAVROS + BARLAS)"
echo "2) Sadece ROS Bridge + Dart Laser"
echo "3) Manuel başlatma (adım adım)"
echo ""

read -p "Seçiminizi yapın (1-3): " choice

case $choice in
    1)
        echo -e "${GREEN}🚀 TAM SİSTEM BAŞLATILIYOR...${NC}"
        
        # Terminal 1: ArduPilot SITL
        echo "Terminal 1: ArduPilot SITL başlatılıyor..."
        gnome-terminal --title="ArduPilot SITL" -- bash -c "
            cd ~/ardupilot
            echo '🎯 ArduPilot SITL Rover başlatılıyor...'
            python3 Tools/autotest/sim_vehicle.py -v Rover --console --out=udp:0.0.0.0:14550
            exec bash
        " &
        
        sleep 5  # SITL'in başlamasını bekle
        
        # Terminal 2: ROS Master + BARLAS Hibrid System
        gnome-terminal --title="BARLAS ROS System" -- bash -c "
            source /opt/ros/noetic/setup.bash
            source ~/barlas_ws/devel/setup.bash
            export ROS_IP=$ROS_IP
            export ROS_MASTER_URI=http://$ROS_IP:11311
            echo '🎯 BARLAS ROS Hibrit Sistem başlatılıyor...'
            roslaunch barlas barlas_hibrid_system.launch
            exec bash
        " &
        
        sleep 3
        
        # Terminal 3: BARLAS Controller Interface
        gnome-terminal --title="BARLAS Controller" -- bash -c "
            source /opt/ros/noetic/setup.bash
            source ~/barlas_ws/devel/setup.bash
            export ROS_IP=$ROS_IP
            export ROS_MASTER_URI=http://$ROS_IP:11311
            echo '🎯 BARLAS Hibrid Controller başlatılıyor...'
            sleep 10  # ROS sisteminin tamamen başlamasını bekle
            python3 ~/barlas_ws/src/barlas/src/barlas_hibrid_controller.py
            exec bash
        " &
        
        ;;
        
    2)
        echo -e "${YELLOW}🎯 SADECE ROS BRIDGE + DART LASER${NC}"
        
        # ROS Master başlat
        gnome-terminal --title="ROS Master" -- bash -c "
            source /opt/ros/noetic/setup.bash
            export ROS_IP=$ROS_IP
            export ROS_MASTER_URI=http://$ROS_IP:11311
            echo '🤖 ROS Master başlatılıyor...'
            roscore
            exec bash
        " &
        
        sleep 3
        
        # Dart Laser System
        gnome-terminal --title="Dart Laser System" -- bash -c "
            source /opt/ros/noetic/setup.bash
            source ~/barlas_ws/devel/setup.bash
            export ROS_IP=$ROS_IP
            export ROS_MASTER_URI=http://$ROS_IP:11311
            echo '🎯 Dart Laser System başlatılıyor...'
            roslaunch barlas_dart_laser dart_laser_system.launch
            exec bash
        " &
        
        ;;
        
    3)
        echo -e "${BLUE}📋 MANUEL BAŞLATMA KOMUTLARI:${NC}"
        echo ""
        echo "Terminal 1 - ArduPilot SITL:"
        echo "cd ~/ardupilot"
        echo "python3 Tools/autotest/sim_vehicle.py -v Rover --console --out=udp:0.0.0.0:14550"
        echo ""
        echo "Terminal 2 - ROS Master:"
        echo "source /opt/ros/noetic/setup.bash"
        echo "export ROS_IP=$ROS_IP"
        echo "export ROS_MASTER_URI=http://$ROS_IP:11311"
        echo "roscore"
        echo ""
        echo "Terminal 3 - MAVROS:"
        echo "source /opt/ros/noetic/setup.bash"
        echo "roslaunch mavros apm.launch fcu_url:=\"udp://:14550@127.0.0.1:14551\""
        echo ""
        echo "Terminal 4 - BARLAS Hibrid System:"
        echo "source ~/barlas_ws/devel/setup.bash"
        echo "roslaunch barlas barlas_hibrid_system.launch"
        echo ""
        exit 0
        ;;
        
    *)
        echo -e "${RED}❌ Geçersiz seçim!${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}✅ BARLAS Hibrit Sistem başlatıldı!${NC}"
echo ""
echo "🔍 SİSTEM KONTROLLERI:"
echo "Mission Planner (Windows): UDP Port 14550'ye bağlan"
echo "ROS Topics: rostopic list"
echo "System Status: rostopic echo /barlas/system/status"
echo "Dart Detection: rostopic echo /barlas/dart_laser/dart_detected"
echo ""
echo "🎮 KONTROL KOMUTLARİ:"
echo "Dart Targeting Aktif: rostopic pub /barlas/dart_laser/targeting_enable std_msgs/Bool \"data: true\""
echo "Emergency Stop: Ctrl+C tüm terminallerde"
echo ""
echo "Press Ctrl+C to stop all processes..."

# Script'in çalışmaya devam etmesini sağla
trap 'echo -e "${RED}🛑 BARLAS Sistem kapatılıyor...${NC}"; exit' INT

# Sonsuz döngü
while true; do
    sleep 1
done
