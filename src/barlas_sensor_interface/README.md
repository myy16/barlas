# BARLAS ROS Sensör Arayüzü 🤖

Bu ROS paketi, Raspberry Pi'den gelen sensör verilerini ROS sistemine entegre eder.

## ROS Paketi Yapısı 📂

```
barlas_sensor_interface/
├── launch/
│   └── sensors.launch    # Sensör arayüz başlatma
├── src/
│   └── sensor_interface_node.py
├── rviz/
│   └── sensors.rviz     # Sensör görselleştirme
├── config/
│   └── params.yaml      # Yapılandırma
├── package.xml          # Paket tanımı
└── CMakeLists.txt      # Derleme ayarları
```

## Bağımlılıklar 📦

- ROS Noetic
- Python 3.x
- python3-serial
- python3-numpy
- std_msgs
- sensor_msgs
- diagnostic_msgs

## Kurulum 🛠️

1. ROS çalışma alanına kopyala:
```bash
cd ~/catkin_ws/src
git clone https://github.com/myy16/barlas.git
```

2. Paketi derle:
```bash
cd ~/catkin_ws
catkin_make
```

3. Kaynak dosyasını yükle:
```bash
source devel/setup.bash
```

## Çalıştırma ▶️

```bash
roslaunch barlas_sensor_interface sensors.launch
```

## ROS Topicleri 📊

### Yayınlanan Topicler

| Topic Adı | Mesaj Tipi | Açıklama |
|-----------|------------|-----------|
| `/sensors/ultrasonic` | Float32MultiArray | 8 ultrasonik sensör verisi |
| `/sensors/lidar` | LaserScan | LIDAR tarama verisi |
| `/sensors/imu` | Imu | IMU oryantasyon ve hareket |
| `/sensors/temperature` | Temperature | Sistem sıcaklığı |
| `/system/battery` | DiagnosticArray | Batarya durumu |

### TF Dönüşümleri

- `base_link` → `base_laser`
- `base_link` → `base_imu`

## Parametreler ⚙️

`sensors.launch` dosyasında ayarlanabilir:

- `serial_port`: Seri port yolu (varsayılan: /dev/ttyACM0)
- `baud_rate`: Haberleşme hızı (varsayılan: 115200)

## RViz Görselleştirme 🎯

1. RViz'i başlat:
```bash
rosrun rviz rviz -d $(rospack find barlas_sensor_interface)/rviz/sensors.rviz
```

2. Görüntülenebilen veriler:
- LIDAR taraması
- Ultrasonik sensör okumaları
- Robot koordinat sistemi

## Sorun Giderme 🔧

### Seri Port Sorunları

1. Port izinlerini kontrol et:
```bash
ls -l /dev/ttyACM*
```

2. Kullanıcıyı dialout grubuna ekle:
```bash
sudo usermod -a -G dialout $USER
```

### ROS Bağlantı Sorunları

1. ROS Master'ı kontrol et:
```bash
rostopic list
```

2. Topic yayınlarını izle:
```bash
rostopic echo /sensors/ultrasonic
```

## Diagnostik Bilgileri 🏥

`rqt_robot_monitor` ile batarya ve sensör durumlarını izle:

```bash
rosrun rqt_robot_monitor rqt_robot_monitor
```

## Performans İyileştirme 🚀

- Topic yayın frekansları `params.yaml`'da ayarlanabilir
- LIDAR veri filtreleme parametreleri özelleştirilebilir
- IMU kalibrasyon değerleri güncellenebilir

## Log Dosyaları 📝

ROS logları:
```bash
~/.ros/log/
```
