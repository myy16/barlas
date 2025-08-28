# BARLAS - TEKNOFEST 2025 İNSANSIZ KARA ARACI YARIŞMASI

Bu proje **Teknofest 2025 İnsansız Kara Aracı Yarışması** için geliştirilmiş kapsamlı bir otonom araç sistemidir.

## 🎯 PROJE GENEL BAKIŞ

**BARLAS (Barış - Autonomous Robot for Land-based Advanced Systems)** hibrit bir mission planner + ROS sistemi kullanarak Teknofest yarışma gereksinimlerini karşılamak için tasarlanmıştır.

### 🏁 Yarışma Görevleri
- **Otonom Sürüş**: Tamamen otonom navigasyon
- **Engel Kaçınma**: LiDAR ve kamera destekli engel tespiti
- **Parkur Görevleri**: 6 farklı parkur elementi
  - Dik Eğim (45°)
  - Yan Eğim (20%)
  - Sığ Su (40cm)
  - Çakıllı Yol
  - Trafik Konileri Slalom
  - Hızlanma Bölgesi
- **Tabela Algılama**: Trafik işaretlerini tanıma ve görev belirleme
- **Dart Atış**: Hedeften dart atışı gerçekleştirme
- **Puanlama Sistemi**: Real-time skor takibi

## 🔧 TEKNİK ÖZELLİKLER

### Donanım Konfigürasyonu
- **Ana Bilgisayar**: MSI Laptop (Intel i5-13420H + RTX 4060)
- **Kontrolcü**: Raspberry Pi 4B (8GB RAM)
- **Flight Controller**: Pixhawk 2.4.8 (ArduPilot)
- **Arduino**: Mega 2560 (Sensör ve servo kontrolü)

### Sensör Donanımı
- **LiDAR**: 360° laser scanner (0.1-10m menzil)
- **Kameralar**: 3x Sony IMX290 (Ana, dart, arka görüş)
- **IMU**: MPU9250 (9-axis inertial measurement)
- **GPS**: Neo-M8N (konum belirleme)
- **Ultrasonik**: 4x JSN-SR04T (yakın mesafe tespiti)
- **Çevre Sensörleri**: BME280 (sıcaklık, nem, basınç)
- **Güç İzleme**: INA219 (batarya durumu)

### Yazılım Mimarisi
- **İşletim Sistemi**: Windows 11 + WSL2 Ubuntu 20.04
- **ROS Sürümü**: ROS Noetic (C++ ve Python)
- **Mission Planner**: ArduPilot GroundStation
- **MAVROS**: ROS-ArduPilot iletişim bridge
- **OpenCV**: Computer vision ve image processing
- **TensorFlow**: Traffic sign recognition AI

## 📁 PROJE YAPISI

```
barlas/
├── README.md                     # Bu dosya
├── CMakeLists.txt               # ROS build konfigürasyonu
├── package.xml                  # ROS paket bilgisi
├── setup.py                     # Python kurulum dosyası
├── config/                      # Konfigürasyon dosyaları
│   └── teknofest_competition_params.yaml
├── dataset/                     # Eğitim verileri
│   ├── labeled_data/           # Etiketlenmiş veriler
│   ├── models/                 # Eğitilmiş AI modelleri
│   └── raw_images/            # Ham görsel veriler
├── docs/                       # Dokümantasyon
│   ├── DETAILED_SETUP.md       # Detaylı kurulum kılavuzu
│   ├── WSL_ROS_SETUP.md       # WSL2 ROS kurulumu
│   ├── DAILY_TEST_PROCEDURE.md # Günlük test prosedürü
│   └── ANA_BILGISAYAR_DEPLOYMENT.md # Deployment kılavuzu
├── launch/                     # ROS launch dosyaları
│   └── barlas_hibrid_system.launch
├── models/                     # 3D modeller ve AI modelleri
│   ├── barlas_simple.urdf     # Robot URDF modeli
│   └── tabela_cnn.pth         # Tabela algılama modeli
├── rviz/                      # RViz görselleştirme
│   └── sensors.rviz           # Sensör görselleştirmesi
├── scripts/                   # Kurulum ve başlatma scriptleri
│   ├── install_main_computer_windows.ps1
│   ├── install_main_computer.sh
│   └── start_barlas_hibrid.sh
├── src/                       # Ana kaynak kodları
│   ├── barlas_hibrid_controller.py    # Ana hibrit controller
│   ├── competition_manager.py         # Yarışma yöneticisi
│   ├── traffic_sign_recognition.py    # Tabela algılama AI
│   ├── parkour_task_manager.py        # Parkur görev yöneticisi
│   ├── debug_utils.py                 # Debug ve test araçları
│   ├── barlas_main_computer_gui.py    # Ana bilgisayar GUI
│   ├── barlas_ros_bridge.py          # ROS bridge
│   ├── lidar_node.py                 # LiDAR işleme
│   ├── ultrasonic_node.py            # Ultrasonik sensörler
│   ├── mission_controller.py         # Mission planner kontrol
│   ├── motor_driver.py               # Motor sürücü
│   ├── obstacle_avoidance.py         # Engel kaçınma
│   ├── pid_controller.py             # PID kontrol algoritması
│   ├── barlas/                       # ROS paketi
│   ├── barlas_sensor_interface/      # Sensör interface paketi
│   ├── dart_laser_system/            # Dart atış sistemi
│   └── sensors/                      # Sensör modülleri
├── raspberry_pi/              # Raspberry Pi kodları
│   ├── sensor_manager.py      # Sensör yöneticisi
│   └── requirements.txt       # Python bağımlılıkları
└── worlds/                    # Gazebo simülasyon dünyaları
    └── barlas.world
```

## 🚀 HIZLI BAŞLANGIÇ

### 1. Sistem Gereksinimleri
- **Windows 11** (WSL2 destekli)
- **WSL2** ile **Ubuntu 20.04 LTS**
- **ROS Noetic** 
- **Mission Planner** (Windows)
- **Python 3.8+**
- **OpenCV 4.5+**
- **TensorFlow 2.x**

### 2. Kurulum

#### Windows Ana Bilgisayar:
```powershell
# PowerShell'i Admin olarak çalıştırın
cd d:\barlas
.\scripts\install_main_computer_windows.ps1
```

#### WSL2 Ubuntu Kurulumu:
```bash
# WSL2'de Ubuntu terminalinde
cd /mnt/d/barlas
./scripts/install_main_computer.sh
```

#### Raspberry Pi Kurulumu:
```bash
# SSH ile Raspberry Pi'ye bağlanın
cd /home/pi/barlas/raspberry_pi
sudo python3 -m pip install -r requirements.txt
```

### 3. Sistem Başlatma

#### Mission Planner (Windows):
1. Mission Planner'ı çalıştırın
2. COM port'u Pixhawk'a bağlayın (genelde COM3-COM8)
3. "CONNECT" butonuna tıklayın
4. Hibrit modu etkinleştirin

#### ROS Sistemi (WSL2):
```bash
# Terminal 1: ROS Master + Sistem
cd /mnt/d/barlas
source /opt/ros/noetic/setup.bash
roslaunch launch/barlas_hibrid_system.launch

# Terminal 2: Hibrit Controller
cd /mnt/d/barlas/src
python3 barlas_hibrid_controller.py

# Terminal 3: Sensör Sistemi  
cd /mnt/d/barlas/src/sensors
python3 test_sensors.py
```

#### Raspberry Pi Sensörler:
```bash
# SSH Terminal
cd /home/pi/barlas/raspberry_pi
python3 sensor_manager.py
```

### 4. Teknofest Yarışması Başlatma

Ana hibrit controller'da:
```
[BARLAS] Komut: competition
🏁 TEKNOFEST YARIŞMASI BAŞLATILDI!
```

## 📋 YARISMA MODÜLLERİ

### Competition Manager (`competition_manager.py`)
- 15 dakikalık yarışma süresi yönetimi
- Real-time skor hesaplama
- 8 farklı yarışma görevi koordinasyonu
- Güvenlik ve emergency stop yönetimi
- GPS tabanlı konum takibi

### Traffic Sign Recognition (`traffic_sign_recognition.py`)
- CNN destekli tabela algılama
- 8 farklı trafik işareti tanıma
- Classical vision fallback
- Stability filtering (3 frame minimum)
- Parkur görev tetikleme

### Parkour Task Manager (`parkour_task_manager.py`)
- **Dik Eğim**: 45° eğim algılama ve geçiş
- **Yan Eğim**: %20 yanal eğim dengesi
- **Sığ Su**: 40cm su derinliğinde seyir
- **Çakıllı Yol**: Vibrasyon algılama ve adaptasyon
- **Trafik Konileri**: Slalom navigasyonu
- **Hızlanma**: Kontrolsüz hızlanma bölgesi

### Dart Laser System (`dart_laser_system/`)
- Arduino Mega destekli pan-tilt servo sistemi
- YOLO object detection ile hedef tespiti
- Laser pointer ile targeting
- Dart atış mekaniği
- ROS node entegrasyonu

### Hibrit Controller (`barlas_hibrid_controller.py`)
- Mission Planner + ROS entegrasyonu
- MAVROS RC override kontrolü
- Competition mode otomatik geçişi
- Emergency stop prosedürleri
- Real-time sistem durumu raporlama

## 🔍 TEST VE DEBUG

### System Debugger
```bash
cd /mnt/d/barlas/src
python3 debug_utils.py
```

Mevcut test fonksiyonları:
- ROS connectivity test
- Sensor health check  
- Competition modules validation
- Navigation stack verification
- Control systems test
- Safety systems check
- Communication test
- File system validation
- Dependencies check

### Sensör Test Procedürleri
```bash
# Tüm sensörler
cd /mnt/d/barlas/src/sensors
python3 test_sensors.py

# Tek sensör testleri
python3 test_lidar.py      # LiDAR testi
python3 test_imu.py        # IMU kalibrasyonu
python3 test_battery.py    # Batarya durumu
python3 test_temperature.py # Sıcaklık sensörleri
```

## 📊 YARISMA PERFORMANSI

### Puanlama Sistemi
- **Temel Puanlar**: Otonom sürüş (20p), Tabela algılama (15p), Dart atış (30p)
- **Parkur Puanları**: Her görev 10-20 puan arası
- **Zaman Bonusu**: Erken bitirme için +10p max
- **Ceza Puanları**: Çarpışma (-5p), Timeout (-2p), Manuel müdahale (-3p)
- **Maksimum Puan**: ~150 puan

### Güvenlik Sistemleri
- **Batarya İzleme**: %15 altında acil durum
- **Sıcaklık İzleme**: 60°C üzerinde sistem koruması
- **Devrilme Koruması**: 50° üzerinde acil durdurma
- **Engel Koruması**: 30cm yakın engelde durdurma
- **Manuel Override**: Her zaman aktif emergency stop

### Performans İzleme
- CPU/Memory/Disk kullanımı izleme
- Sensör sağlık durumu tracking
- Network latency ölçümü
- Real-time error logging
- Competition metrics dashboard

## 🔗 BAĞLANTILAR VE REFERANSLAR

### Teknofest Resmi Kaynaklar
- [Teknofest Resmi Sitesi](https://www.teknofest.org/)
- [İnsansız Kara Aracı Yarışması Kuralları](https://www.teknofest.org/yarisma/insansiz-kara-araci/)

### Teknik Dokümantasyon
- [ArduPilot Dokümantasyonu](https://ardupilot.org/)
- [ROS Noetic Tutorials](http://wiki.ros.org/noetic)
- [MAVROS Usage](http://wiki.ros.org/mavros)
- [Mission Planner](https://ardupilot.org/planner/)

### Hardware Referansları
- [Pixhawk 2.4.8](https://docs.px4.io/main/en/flight_controller/pixhawk.html)
- [Raspberry Pi 4](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/)
- [Arduino Mega](https://www.arduino.cc/en/Main/ArduinoBoardMega2560)

## 🏆 TAKIM BİLGİLERİ

**Takım Adı**: BARLAS Team  
**Yarışma**: Teknofest 2025 İnsansız Kara Aracı Yarışması  
**Kategori**: Üniversite  
**Geliştirme Süresi**: 2024-2025  

### Ekip Rolleri
- **Sistem Mimarisi**: Hibrit ROS+Mission Planner entegrasyonu
- **Yapay Zeka**: Traffic sign recognition ve object detection  
- **Donanım Entegrasyonu**: Sensör fusion ve electronic integration
- **Kontrol Algoritmaları**: PID control ve autonomous navigation
- **Test ve Validasyon**: System testing ve performance optimization

## 📝 LİSANS

Bu proje Teknofest 2025 İnsansız Kara Aracı Yarışması için geliştirilmiştir.
Akademik ve eğitim amaçlı kullanım için açık kaynak olarak sunulmaktadır.

---

**Son Güncelleme**: 27 Aralık 2024  
**Versiyon**: 2.0.0  
**Durum**: Yarışma Hazır ✅

---

### 🚨 HIZLI DEStek

Sistem sorunları için:
1. `debug_utils.py` çalıştırarak tanı raporu alın
2. Log dosyalarını kontrol edin: `/tmp/barlas_logs/`
3. Sensör testlerini çalıştırın: `test_sensors.py`  
4. ROS topic'leri kontrol edin: `rostopic list`
5. MAVROS bağlantısını doğrulayın: `rostopic echo /mavros/state`

**Başarılı yarışmalar dileriz! 🏁🏆**
