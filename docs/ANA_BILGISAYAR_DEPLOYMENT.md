# BARLAS Ana Bilgisayar Deployment Rehberi
# MSI Laptop i5-13420H + RTX 4060 için Hibrit Sistem Kurulum

## 🎯 Sistem Gereksinimleri
- **Donanım**: MSI Laptop i5-13420H + RTX 4060
- **İşletim Sistemi**: Windows 11 + WSL2 Ubuntu 20.04
- **RAM**: Minimum 16GB (32GB önerilir)
- **Depolama**: Minimum 100GB boş alan

## 📦 Kurulum Sırası

### 1. Windows Hazırlık (PowerShell Admin)
```powershell
# PowerShell'i yönetici olarak çalıştırın
cd D:\barlas\scripts
.\install_main_computer_windows.ps1 -All
```

**Bu script şunları yapar:**
- WSL2 ve Ubuntu 20.04 kurulumu
- Network port forwarding (5760, 14550, 11311)
- Windows Firewall konfigürasyonu  
- Mission Planner kurulumu

### 2. WSL2 Ubuntu Kurulum
```bash
# WSL2'ye girin
wsl

# BARLAS kurulum scriptini çalıştırın
cd /mnt/d/barlas/scripts
chmod +x install_main_computer.sh
./install_main_computer.sh
```

**Bu script şunları kurar:**
- ROS Noetic Desktop Full
- MAVROS + GeographicLib
- NVIDIA CUDA Toolkit (GPU desteği)
- ArduPilot SITL + Gazebo
- Python ML Stack (PyTorch, YOLO)
- BARLAS ROS Workspace

### 3. Sistem Test
```bash
# Terminal 1: ArduPilot SITL
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v Rover --console --out=udp:0.0.0.0:14550

# Terminal 2: BARLAS ROS System  
source ~/barlas_ws/devel/setup.bash
roslaunch barlas barlas_hibrid_system.launch

# Windows: Mission Planner
# Connection: UDP, Port: 14550, Connect
```

## 🏗️ Sistem Mimarisi

### Ana Bilgisayar Rolleri:
```
┌─────────────────────────────────────────────────────────┐
│                MSI Laptop (Ana Bilgisayar)             │
├─────────────────────────────────────────────────────────┤
│  Windows 11                                             │
│  ├── Mission Planner (Kontrol & Monitoring)            │
│  └── WSL2 Ubuntu 20.04                                 │
│      ├── ROS Noetic (Ana Kontrol Merkezi)              │
│      ├── BARLAS Hibrit Controller                      │
│      ├── Dart Laser System (YOLO + Pan-Tilt)          │
│      ├── ArduPilot SITL (Simülasyon)                   │
│      └── Gazebo (3D Simülasyon)                        │
└─────────────────────────────────────────────────────────┘
                         │ USB Serial
                         ▼
┌─────────────────────────────────────────────────────────┐
│                 Raspberry Pi 4B                        │
├─────────────────────────────────────────────────────────┤
│  ├── 8x JSN-SR04T Ultrasonik Sensör                    │
│  ├── YDLIDAR X4 (360° LIDAR)                           │
│  ├── MPU9250 IMU                                       │
│  ├── DS18B20 Sıcaklık Sensörü                          │
│  └── INA219 Batarya Monitörü                           │
└─────────────────────────────────────────────────────────┘
                         │ Serial/GPIO
                         ▼
┌─────────────────────────────────────────────────────────┐
│                Arduino Pan-Tilt System                 │  
├─────────────────────────────────────────────────────────┤
│  ├── 2x SG90 Servo (Pan-Tilt)                          │
│  ├── USB Kamera (640x480)                              │
│  └── Lazer Pointer Modülü                              │
└─────────────────────────────────────────────────────────┘
```

### Veri Akışı:
```
Raspberry Pi Sensörler → JSON → USB Serial → Ana Bilgisayar → ROS Topics
Kamera → YOLO (GPU) → Dart Tespiti → Pan-Tilt Arduino → Lazer Hedefleme  
ROS → MAVROS → Mission Planner → RC Override → ArduPilot → Motor Kontrol
```

## 🚀 Günlük Başlatma Prosedürü

### Hızlı Başlatma (3 Terminal):
```bash
# Terminal 1: Sistem başlatma scripti
cd ~/barlas/scripts  
./start_barlas_hibrid.sh

# Windows: Mission Planner
# UDP 14550'ye bağlan
```

### Manuel Başlatma:
```bash
# Terminal 1: ArduPilot SITL
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v Rover --out=udp:0.0.0.0:14550

# Terminal 2: ROS Master + BARLAS
source ~/barlas_ws/devel/setup.bash
roslaunch barlas barlas_hibrid_system.launch

# Terminal 3: BARLAS Controller
python3 ~/barlas_ws/src/barlas/src/barlas_hibrid_controller.py
```

## 🎮 Kontrol Komutları

### ROS Topic'ler:
```bash
# Sistem durumu
rostopic echo /barlas/system/status

# Dart tespiti
rostopic echo /barlas/dart_laser/dart_detected  

# Sensör verileri
rostopic echo /barlas/sensors/ultrasonic
rostopic echo /barlas/sensors/imu

# Manuel kontrol
rostopic pub /barlas/dart_laser/targeting_enable std_msgs/Bool "data: true"
```

### Mission Planner:
- **Connection**: UDP Port 14550
- **Vehicle Type**: Rover
- **Flight Mode**: MANUAL, AUTO, GUIDED
- **RC Override**: Otonom kontrol için

## 🔧 Troubleshooting

### WSL2 Network Sorunları:
```powershell
# Windows PowerShell (Admin)
wsl --shutdown
wsl
netsh interface portproxy show all
```

### ROS Bağlantı Sorunları:
```bash
# WSL2'de network kontrol
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://$ROS_IP:11311
rostopic list
```

### CUDA GPU Desteği:
```bash
# GPU kontrolü
nvidia-smi
nvcc --version

# PyTorch CUDA test
python3 -c "import torch; print(torch.cuda.is_available())"
```

## 📊 Performans Hedefleri

- **YOLO Inference**: 30+ FPS (RTX 4060)
- **ROS Topic Frequency**: 10-50 Hz
- **MAVROS Latency**: <50ms  
- **Mission Planner Update**: 1-5 Hz
- **Total System Startup**: <3 dakika

## 🎯 Deployment Checklist

**Windows Tarafı:**
- [ ] WSL2 etkin ve Ubuntu 20.04 kurulu
- [ ] Mission Planner kurulu ve çalışıyor
- [ ] Port forwarding kuralları aktif
- [ ] Windows Firewall konfigüre

**WSL2 Ubuntu Tarafı:**  
- [ ] ROS Noetic desktop-full kurulu
- [ ] MAVROS kurulu ve çalışıyor
- [ ] BARLAS workspace build edilmiş
- [ ] ArduPilot SITL çalışıyor
- [ ] CUDA/GPU desteği aktif

**Hardware Bağlantıları:**
- [ ] Raspberry Pi USB serial bağlı
- [ ] Arduino pan-tilt USB bağlı  
- [ ] USB kamera bağlı
- [ ] Tüm sensörler çalışıyor

✅ **Sistem hazır - Teknofest yarışmasına katılabilir!**
