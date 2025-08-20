# WSL ve ROS Kurulum Rehberi 🚀

Bu rehber, Windows'ta WSL2 üzerinde ROS ve Gazebo simülasyon ortamının kurulumunu adım adım açıklar.

## 1. WSL2 Kurulumu 🐧

### Windows'u Hazırlama
1. PowerShell'i yönetici olarak açın
2. WSL'yi etkinleştirin:
```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```
3. Bilgisayarınızı yeniden başlatın
4. WSL2 Linux kernel güncellemesini indirin ve kurun:
   - https://wslstorecdn.microsoft.com/wsl/wsl_update_x64.msi

5. WSL2'yi varsayılan sürüm yapın:
```powershell
wsl --set-default-version 2
```

### Ubuntu 20.04 Kurulumu
1. Microsoft Store'dan "Ubuntu 20.04 LTS"yi indirin ve kurun
2. Ubuntu'yu başlatın ve kullanıcı adı/şifre oluşturun
3. Sistemi güncelleyin:
```bash
sudo apt update
sudo apt upgrade -y
```

## 2. X Server Kurulumu 🖥️

### VcXsrv Kurulumu
1. VcXsrv'yi indirin:
   - https://sourceforge.net/projects/vcxsrv/

2. Kurulumu tamamlayın

3. XLaunch'ı çalıştırın:
   - ✅ "Multiple windows" seçin
   - ✅ "Start no client" seçin
   - ✅ "Clipboard" işaretleyin
   - ✅ "Primary Selection" işaretleyin
   - ✅ "Native opengl" işaretleyin
   - ✅ "Disable access control" işaretleyin
   - "Finish" tıklayın

4. WSL'de grafik ayarlarını yapın:
```bash
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc
source ~/.bashrc
```

## 3. ROS Noetic Kurulumu 🤖

1. ROS deposunu ekleyin:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

2. Sistemi güncelleyin:
```bash
sudo apt update
```

3. ROS Noetic'i kurun:
```bash
sudo apt install -y ros-noetic-desktop-full
```

4. ROS ortam değişkenlerini ekleyin:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

5. ROS bağımlılıklarını kurun:
```bash
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

6. rosdep'i başlatın:
```bash
sudo rosdep init
rosdep update
```

## 4. Gazebo ve MAVROS Kurulumu 🎮

1. Gazebo ve ROS paketlerini kurun:
```bash
sudo apt install -y gazebo11 libgazebo11-dev
sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

2. MAVROS'u kurun:
```bash
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## 5. Çalışma Alanı Oluşturma 📂

1. ROS workspace'i oluşturun:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. BARLAS paketlerini klonlayın:
```bash
git clone https://github.com/myy16/barlas.git
```

3. Workspace'i derleyin:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 6. QGroundControl Kurulumu 🛩️

1. Bağımlılıkları kurun:
```bash
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

2. QGroundControl'u indirin ve kurun:
```bash
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
```

## Sorun Giderme 🔧

### WSL Grafik Sorunları
- XLaunch'ın çalıştığından emin olun
- Windows Defender Firewall'da XServer'a izin verin
- WSL'de DISPLAY değişkenini kontrol edin:
```bash
echo $DISPLAY
```

### ROS Sorunları
- ROS ortam değişkenlerini kontrol edin:
```bash
printenv | grep ROS
```

### Gazebo Sorunları
- Gazebo'yu yeniden yükleyin:
```bash
sudo apt remove --purge gazebo*
sudo apt install gazebo11 libgazebo11-dev
```

## Başlangıç Kontrolleri ✅

1. ROS'un çalıştığını kontrol edin:
```bash
roscore
```

2. Gazebo'yu test edin:
```bash
gazebo
```

3. MAVROS durumunu kontrol edin:
```bash
rostopic list | grep mavros
```

## Yardımcı Notlar 📝

- WSL'i yeniden başlatma:
```powershell
wsl --shutdown
```

- Ubuntu'yu güncelleme:
```bash
sudo apt update && sudo apt upgrade -y
```

- ROS workspace'i yeniden derleme:
```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
source devel/setup.bash
```
