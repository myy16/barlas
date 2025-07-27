# BARLAS Dart Laser System - ROS Integration

🎯 **YOLO dart detection + Pan-Tilt laser targeting + ROS integration**

Bu sistem BARLAS aracının ROS ile dart tanıma ve lazer hedefleme yapmasını sağlar.

## 📁 ROS Dosya Yapısı

```
ros_nodes/
├── dart_laser_ros_node.py          # Ana ROS node
├── servo_controller_ros_node.py    # Servo kontrol node
├── ros_test_script.py              # ROS test scripti
├── launch/
│   └── dart_laser_system.launch    # Launch dosyası
├── rviz/
│   └── dart_laser_viz.rviz         # RViz konfigürasyonu
├── CMakeLists.txt                   # CMake build dosyası
├── package.xml                      # ROS paket tanımı
└── README_ROS.md                    # Bu dosya
```

## 🚀 Linux'ta Hızlı Kurulum

```bash
# Kurulum scriptini çalıştır
cd dart_laser_system
chmod +x linux_setup_test.sh
./linux_setup_test.sh
```

## 🔌 Donanım Bağlantıları

| Bileşen | GPIO Pin | Fiziksel Pin | Açıklama |
|---------|----------|--------------|----------|
| Pan Servo | GPIO 18 | Pin 12 | Yatay hareket |
| Tilt Servo | GPIO 19 | Pin 35 | Dikey hareket |
| Lazer Diod | GPIO 20 | Pin 38 | Lazer pointer |
| 5V | 5V Power | Pin 2/4 | Servo besleme |
| GND | Ground | Pin 6/9/14 | Ortak topraklama |

## 🤖 ROS Topic'leri

### Publishers (Sistem Çıkışları)
- `/barlas/dart_laser/image_result` (Image) - İşlenmiş kamera görüntüsü
- `/barlas/dart_laser/dart_detected` (PointStamped) - Tespit edilen dart koordinatları
- `/barlas/servo/current_position` (Float32MultiArray) - Servo pozisyonu [pan, tilt]
- `/barlas/servo/laser_status` (Bool) - Lazer durumu
- `/barlas/dart_laser/system_status` (String) - Sistem durumu

### Subscribers (Sistem Girişleri)
- `/barlas/camera/image_raw` (Image) - Kamera görüntüsü
- `/barlas/servo/position_command` (Float32MultiArray) - Servo pozisyon komutu
- `/barlas/servo/laser_command` (Bool) - Lazer kontrol komutu
- `/barlas/servo/pixel_target` (Point) - Piksel hedef komutu
- `/barlas/dart_laser/targeting_enable` (Bool) - Hedefleme sistemi aktif/pasif

## 🎮 Manuel Kontrol Komutları

### Servo Kontrolü
```bash
# Merkez pozisyon
rostopic pub /barlas/servo/center_command std_msgs/Bool "data: true"

# Belirli pozisyon (pan=45°, tilt=60°)
rostopic pub /barlas/servo/position_command std_msgs/Float32MultiArray "data: [45, 60]"

# Piksel hedefleme (x=320, y=240)
rostopic pub /barlas/servo/pixel_target geometry_msgs/Point "x: 320.0
y: 240.0
z: 0.0"
```

### Lazer Kontrolü
```bash
# Lazer aç
rostopic pub /barlas/servo/laser_command std_msgs/Bool "data: true"

# Lazer kapat
rostopic pub /barlas/servo/laser_command std_msgs/Bool "data: false"
```

### Dart Hedefleme
```bash
# Hedefleme sistemi aktif
rostopic pub /barlas/dart_laser/targeting_enable std_msgs/Bool "data: true"

# Hedefleme sistemi pasif
rostopic pub /barlas/dart_laser/targeting_enable std_msgs/Bool "data: false"
```

## 📊 İzleme Komutları

```bash
# Topic listesi
rostopic list

# Servo pozisyonu izle
rostopic echo /barlas/servo/current_position

# Dart tespitlerini izle
rostopic echo /barlas/dart_laser/dart_detected

# Lazer durumunu izle
rostopic echo /barlas/servo/laser_status

# Sistem durumunu izle
rostopic echo /barlas/dart_laser/system_status
```

## 🖥️ Görsel İzleme

### RQT ile Kamera Görüntüsü
```bash
# Kamera görüntüsü
rqt_image_view

# Topic seç: /barlas/dart_laser/image_result
```

### RViz ile 3D Görselleştirme
```bash
rviz -d $(rospack find barlas_dart_laser)/rviz/dart_laser_viz.rviz
```

## 🧪 Test Prosedürleri

### 1. Temel Sistem Testi
```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: Servo Controller
rosrun barlas_dart_laser servo_controller_ros_node.py

# Terminal 3: Test Script
rosrun barlas_dart_laser ros_test_script.py
```

### 2. Komple Sistem Testi
```bash
# Tek komutla tüm sistemi başlat
roslaunch barlas_dart_laser dart_laser_system.launch

# Test script çalıştır
rosrun barlas_dart_laser ros_test_script.py dart
```

### 3. İnteraktif Test
```bash
# İnteraktif test menüsü
rosrun barlas_dart_laser ros_test_script.py

# Manuel servo kontrolü için 'w/s/a/d' tuşları
# 'l' tuşu ile lazer aç/kapat
# 'c' tuşu ile merkez pozisyon
```

## 🔧 Troubleshooting

### ROS Node Başlatılamıyor
```bash
# ROS environment source et
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Node'u tekrar başlat
rosrun barlas_dart_laser servo_controller_ros_node.py
```

### GPIO İzin Sorunu
```bash
# Kullanıcıyı gpio grubuna ekle
sudo usermod -a -G gpio $USER

# Çıkış yap ve tekrar giriş yap
```

### Kamera Bulunamıyor
```bash
# Kamera cihazları kontrol et
ls /dev/video*

# V4L2 bilgileri
v4l2-ctl --list-devices

# Farklı kamera indeksi dene
roslaunch barlas_dart_laser dart_laser_system.launch camera_index:=1
```

### Servo Hareket Etmiyor
```bash
# GPIO durumu kontrol et
sudo dmesg | grep gpio

# Servo besleme voltajı kontrol et (5V)
# PWM sinyali kontrol et (oscilloscope ile)

# Servo kablo bağlantıları:
# Kırmızı -> 5V (Pin 2/4)
# Kahverengi -> GND (Pin 6/9/14)  
# Turuncu -> GPIO (Pin 12: GPIO18, Pin 35: GPIO19)
```

## 📈 Performans Optimizasyonu

### CPU Kullanımını Azaltma
```bash
# YOLO çözünürlüğünü düşür
rosparam set /dart_laser_node/yolo_input_size 416

# Kamera FPS'i düşür
rosparam set /usb_cam/framerate 15

# Frame skip aktif et
rosparam set /dart_laser_node/frame_skip 2
```

### Network Latency Azaltma
```bash
# Compressed image kullan
rostopic echo /barlas/dart_laser/image_compressed

# Büyük topic'leri local işle
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

## 🔄 Konfigürasyon Parametreleri

Launch dosyasında değiştirilebilir parametreler:

```xml
<arg name="camera_index" default="0" />           <!-- Kamera indeksi -->
<arg name="confidence_threshold" default="0.5" /> <!-- Dart güven eşiği -->
<arg name="lock_time" default="1.5" />            <!-- Kilitlenme süresi -->
<arg name="laser_duration" default="3.0" />       <!-- Lazer açık kalma süresi -->
<arg name="pan_pin" default="18" />               <!-- Pan servo GPIO -->
<arg name="tilt_pin" default="19" />              <!-- Tilt servo GPIO -->
<arg name="laser_pin" default="20" />             <!-- Lazer GPIO -->
```

Özel parametrelerle başlatma:
```bash
roslaunch barlas_dart_laser dart_laser_system.launch confidence_threshold:=0.7 lock_time:=1.0
```

## 🎯 Kullanım Senaryoları

### 1. Otomatik Dart Hedefleme
- Kamera görüntüsünden dart tespit
- Pan-tilt ile kamerayı hedefe yönlendir  
- Lazer ile dart'ı işaretle
- ROS topic'leri ile durumu bildir

### 2. Manuel Dart İşaretleme
- RViz'de dart koordinatlarını tıkla
- Servo otomatik o noktaya yönelir
- Lazer manuel olarak açılır

### 3. Web Interface Kontrolü
- ROS web interface ile kontrol
- Topic'leri web sayfasından publish et
- Kamera görüntüsünü web'de görüntüle

---

🎯 **BARLAS Dart Laser ROS System v1.0**  
*Jupyter notebook'tan ROS node'larına tam entegrasyon*
