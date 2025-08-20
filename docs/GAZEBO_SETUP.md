# PX4 SITL ve Gazebo Kurulum Rehberi

## Sistem Gereksinimleri
- Ubuntu 20.04 (WSL2 üzerinde)
- ROS Noetic
- Gazebo 11
- PX4 Firmware
- QGroundControl

## Kurulum Adımları

### 1. ROS Noetic Kurulumu (Eğer kurulu değilse)
```bash
# ROS repository ekleme
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ROS kurulumu
sudo apt update
sudo apt install ros-noetic-desktop-full

# ROS environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ROS dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### 2. PX4 Firmware Kurulumu
```bash
# Gerekli araçların kurulumu
sudo apt install git python3-pip python3-jinja2 python3-setuptools python3-toml python3-packaging

# PX4 Firmware indirme
mkdir -p ~/PX4-Autopilot
cd ~/PX4-Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git .

# PX4 dependencies kurulumu
bash ./Tools/setup/ubuntu.sh

# PX4 derleme
make px4_sitl gazebo
```

### 3. QGroundControl Kurulumu
```bash
# Dependencies
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

# QGroundControl indirme ve kurulum
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
```

### 4. MAVROS Kurulumu
```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## Test ve Doğrulama

1. Yeni bir terminal açın ve PX4 SITL'i başlatın:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

2. Başka bir terminalde MAVROS'u başlatın:
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

3. QGroundControl'u başlatın:
```bash
./QGroundControl.AppImage
```

## Olası Sorunlar ve Çözümleri

1. Gazebo Çarpışma Hatası:
```bash
# Gazebo'yu yeniden yükleme
sudo apt remove --purge gazebo*
sudo apt install gazebo11 libgazebo11-dev
```

2. MAVROS Bağlantı Hatası:
```bash
# FCU bağlantı parametrelerini kontrol et
rostopic echo /mavros/state
```

3. QGC Bağlantı Sorunu:
- UDP portlarının açık olduğundan emin olun
- Firewall ayarlarını kontrol edin
