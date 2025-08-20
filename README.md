# BARLAS Robot Sensör Sistemi 🤖

Bu repo, BARLAS otonom kara aracının sensör sistemi ve veri işleme altyapısını içerir.

## Sistem Mimarisi 🏗️

```
BARLAS Sensör Sistemi
├── Raspberry Pi (Sensör Yönetimi)
│   ├── Ultrasonik Sensörler (x8)
│   ├── YDLIDAR X4
│   ├── MPU9250 IMU
│   ├── DS18B20 Sıcaklık
│   └── INA219 Batarya Monitörü
│
└── MSI Laptop (ROS Master)
    └── Sensör Verileri İşleme
```

## Bağlantı Şeması 🔌

- **Ultrasonik Sensörler**: GPIO pinleri üzerinden bağlantı
- **LIDAR**: USB bağlantı
- **IMU**: I2C bağlantı
- **Sıcaklık Sensörü**: 1-Wire bağlantı
- **Batarya Monitörü**: I2C bağlantı

## Sensör Konumları 📍

```
    Ön
[1][2][3][4]  
└─────────┘
│         │
│         │  1-4: Ön ultrasonik sensörler
│         │  5-8: Arka ultrasonik sensörler
│         │  L: LIDAR (merkez)
│         │  I: IMU (merkez)
└─────────┘
[5][6][7][8]
    Arka
```

## Kurulum 🛠️

### Raspberry Pi Kurulumu

1. Gerekli paketleri yükle:
```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install pyserial numpy RPi.GPIO
```

2. I2C ve 1-Wire arayüzlerini etkinleştir:
```bash
sudo raspi-config
# Interfacing Options > I2C > Yes
# Interfacing Options > 1-Wire > Yes
```

3. Sensör yöneticisini başlat:
```bash
python3 sensor_manager.py
```

### MSI Laptop (WSL) Kurulumu

1. ROS paketini derle:
```bash
cd ~/catkin_ws
catkin_make
```

2. Sensör arayüzünü başlat:
```bash
source devel/setup.bash
roslaunch barlas_sensor_interface sensors.launch
```

## Veri Akışı 📊

### ROS Topicleri

- `/sensors/ultrasonic` - Ultrasonik sensör verileri
- `/sensors/lidar` - LIDAR tarama verileri
- `/sensors/imu` - IMU verileri
- `/sensors/temperature` - Sıcaklık verisi
- `/system/battery` - Batarya durumu

### Veri Formatları

- **Ultrasonik**: Float32MultiArray (8 değer)
- **LIDAR**: LaserScan
- **IMU**: Imu
- **Sıcaklık**: Temperature
- **Batarya**: DiagnosticArray

## Sorun Giderme 🔧

### Seri Port Bağlantı Sorunları

1. Port izinlerini kontrol et:
```bash
sudo chmod 666 /dev/ttyACM0
```

2. USB bağlantısını kontrol et:
```bash
ls -l /dev/ttyACM*
```

### Sensör Sorunları

- **Ultrasonik Sensörler**: GPIO pinlerini ve kablolamayı kontrol et
- **LIDAR**: USB bağlantısını ve güç kaynağını kontrol et
- **IMU**: I2C adresini ve bağlantıyı kontrol et
- **Sıcaklık**: 1-Wire arayüzünü kontrol et

## Bakım 🔍

- Her kullanımdan önce sensörlerin fiziksel durumunu kontrol et
- Düzenli olarak sensör kalibrasyonlarını yap
- Batarya voltajını sürekli izle
- Bağlantı kablolarını düzenli kontrol et

## Güvenlik Önlemleri ⚠️

1. Batarya voltajı 10V'un altına düşerse sistemi kapat
2. Sensörlerden aşırı sıcaklık tespit edilirse uyar
3. Bağlantı kopması durumunda güvenli duruş moduna geç

## İletişim 📧

Sorunlar ve öneriler için: myy16@gmail.com
