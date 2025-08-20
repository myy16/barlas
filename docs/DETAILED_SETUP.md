# BARLAS Gazebo Simülasyon Kurulum - Adım Adım Rehber

## 1. ROS Noetic Kurulumu

```bash
# ROS Noetic deposunu ekle
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Sistem güncellemesi
sudo apt update
sudo apt upgrade -y

# ROS Noetic kurulumu
sudo apt install -y ros-noetic-desktop-full

# ROS bağımlılıkları
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# ROS environment kurulumu
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep başlatma
sudo rosdep init
rosdep update
```

## 2. BARLAS ROS Workspace Kurulumu

```bash
# Workspace oluşturma
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# BARLAS repo klonlama
git clone https://github.com/myy16/barlas.git

# Workspace derleme
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 3. PX4 Firmware Kurulumu

```bash
# Gerekli araçların kurulumu
sudo apt install -y git python3-pip python3-jinja2 python3-setuptools python3-toml python3-packaging ninja-build

# PX4 Firmware indirme
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# PX4 bağımlılıklarını kurma
bash ./Tools/setup/ubuntu.sh

# PX4 derleme (bu biraz zaman alabilir)
make px4_sitl_default
```

## 4. MAVROS Kurulumu

```bash
# MAVROS ve gerekli paketler
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras

# GeographicLib datasets kurulumu
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## 5. Gazebo ve Ek Bağımlılıklar

```bash
# Gazebo ve gerekli ROS paketleri
sudo apt install -y gazebo11 libgazebo11-dev
sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Ek ROS paketleri
sudo apt install -y ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
sudo apt install -y ros-noetic-robot-state-publisher ros-noetic-xacro
```

## 6. QGroundControl Kurulumu

```bash
# QGC bağımlılıkları
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

# QGroundControl indirme ve kurulum
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
```

## 7. BARLAS Simülasyonunu Çalıştırma

Terminal 1 - PX4 SITL:
```bash
cd ~/PX4-Autopilot
source ~/catkin_ws/devel/setup.bash
make px4_sitl gazebo
```

Terminal 2 - BARLAS Launch:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch barlas simulation.launch
```

Terminal 3 - QGroundControl:
```bash
cd ~
./QGroundControl.AppImage
```

## 8. Olası Sorunlar ve Çözümleri

### WSL Grafik Sorunu
Windows'ta X Server kurulumu gereklidir:
1. VcXsrv indir ve kur: https://sourceforge.net/projects/vcxsrv/
2. WSL'de:
```bash
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc
source ~/.bashrc
```

### Gazebo Çarpışma Sorunu
```bash
sudo apt remove --purge gazebo*
sudo apt install gazebo11 libgazebo11-dev
```

### PX4 Derleme Hatası
```bash
cd ~/PX4-Autopilot
make distclean
git submodule update --init --recursive
make px4_sitl_default
```

### MAVROS Bağlantı Hatası
```bash
# FCU bağlantı durumunu kontrol et
rostopic echo /mavros/state
```

## 9. Faydalı ROS Komutları

```bash
# Topic'leri listele
rostopic list

# Node'ları listele
rosnode list

# MAVROS durumunu kontrol et
rostopic echo /mavros/state

# TF ağacını görüntüle
rosrun rqt_tf_tree rqt_tf_tree
```
