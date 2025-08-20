# BARLAS Günlük Test Prosedürü 🔄

Bu dokümanda, BARLAS simülasyonunu günlük olarak başlatmak ve test etmek için gereken adımlar açıklanmaktadır.

## 1. Sistem Başlatma 🚀

### X Server Başlatma
1. XLaunch programını çalıştırın
2. Ayarları kontrol edin:
   - Multiple windows ✓
   - Display number: 0
   - Start no client ✓
   - Disable access control ✓

### WSL Terminali Açma
1. Windows Terminal'i açın
2. Ubuntu 20.04 sekmesini seçin
3. Grafik ayarlarını kontrol edin:
```bash
echo $DISPLAY  # :0 olmalı
```

## 2. ROS ve Gazebo Başlatma 🤖

### Terminal 1 - ROS Core
```bash
# ROS ortamını yükle
source ~/catkin_ws/devel/setup.bash

# ROS Core'u başlat
roscore
```

### Terminal 2 - PX4 SITL
```bash
# PX4 klasörüne git
cd ~/PX4-Autopilot

# SITL'i başlat
make px4_sitl gazebo
```

### Terminal 3 - BARLAS Simülasyon
```bash
# ROS ortamını yükle
source ~/catkin_ws/devel/setup.bash

# Simülasyonu başlat
roslaunch barlas simulation.launch
```

### Terminal 4 - QGroundControl
```bash
# QGC'yi başlat
cd ~
./QGroundControl.AppImage
```

## 3. Sistem Kontrolü ✅

### ROS Durumu
```bash
# Topic'leri listele
rostopic list

# Node'ları kontrol et
rosnode list

# TF ağacını görüntüle
rosrun rqt_tf_tree rqt_tf_tree
```

### MAVROS Bağlantısı
```bash
# MAVROS durumunu kontrol et
rostopic echo /mavros/state
```

### Sensör Verileri
```bash
# IMU verilerini kontrol et
rostopic echo /mavros/imu/data

# GPS verilerini kontrol et
rostopic echo /mavros/global_position/global
```

## 4. Test Uçuşu 🛫

1. QGroundControl'de:
   - Bağlantıyı kontrol et
   - Uçuş modunu "OFFBOARD" seç
   - Arm butonuna tıkla

2. Test node'unu çalıştır:
```bash
# Yeni bir terminal aç
source ~/catkin_ws/devel/setup.bash
rosrun barlas_control px4_control_node.py
```

## 5. Kapatma Prosedürü 🛑

1. Tüm ROS node'larını durdur:
```bash
rosnode kill -a
```

2. Gazebo'yu kapat:
   - Terminal 2'de Ctrl+C

3. QGroundControl'u kapat

4. X Server'ı kapat:
   - XLaunch'ı sonlandır

## Sorun Giderme 🔧

### Gazebo Açılmıyorsa
```bash
# Gazebo'yu yeniden yükle
sudo apt remove --purge gazebo*
sudo apt install gazebo11 libgazebo11-dev
```

### MAVROS Bağlantı Sorunu
```bash
# MAVROS'u yeniden başlat
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### Grafik Sorunu
```bash
# X Server bağlantısını kontrol et
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=0
```

## Önemli Notlar 📝

- Her test öncesi batarya seviyesini kontrol edin
- Acil durum prosedürlerini hazır tutun
- Test sonuçlarını kaydedin
- Hataları raporlayın

## Yararlı Komutlar 💻

```bash
# WSL'i yeniden başlat
wsl --shutdown

# ROS log temizleme
rosclean purge

# Çalışma alanını temizle
cd ~/catkin_ws
catkin_make clean
```
