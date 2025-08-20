# Raspberry Pi Sensör Yöneticisi 📊

Bu klasör, BARLAS robotunun Raspberry Pi üzerinde çalışan sensör yönetim sistemini içerir.

## Yazılım Bileşenleri 🔧

- `sensor_manager.py`: Ana sensör yönetim programı
- Sensör sürücüleri ve yardımcı modüller

## Sensör Pinleri 📌

### Ultrasonik Sensörler

| Sensör No | Konum     | Trigger Pin | Echo Pin |
|-----------|-----------|-------------|-----------|
| 1         | Ön-Sol    | GPIO17      | GPIO27    |
| 2         | Ön-Orta-S | GPIO22      | GPIO23    |
| 3         | Ön-Orta-G | GPIO24      | GPIO25    |
| 4         | Ön-Sağ    | GPIO5       | GPIO6     |
| 5         | Arka-Sol  | GPIO13      | GPIO19    |
| 6         | Arka-O-S  | GPIO20      | GPIO21    |
| 7         | Arka-O-G  | GPIO12      | GPIO16    |
| 8         | Arka-Sağ  | GPIO26      | GPIO18    |

### I2C Cihazları

- MPU9250 IMU: Adres 0x68
- INA219 Batarya Monitörü: Adres 0x40

### Diğer Bağlantılar

- YDLIDAR X4: USB (/dev/ttyUSB0)
- DS18B20 Sıcaklık: 1-Wire (GPIO4)

## Kurulum 🛠️

1. Gerekli Python paketlerini yükle:
```bash
pip3 install -r requirements.txt
```

2. GPIO izinlerini ayarla:
```bash
sudo usermod -a -G gpio $USER
```

3. I2C ve 1-Wire'ı etkinleştir:
```bash
# /boot/config.txt'ye ekle:
dtoverlay=i2c-bcm2708
dtoverlay=w1-gpio
```

## Çalıştırma ▶️

```bash
python3 sensor_manager.py
```

## Veri Formatı 📋

Seri port üzerinden gönderilen JSON formatı:

```json
{
    "timestamp": 1629456789.123,
    "ultrasonics": [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2],
    "lidar": [...],
    "imu": {
        "orientation": [w, x, y, z],
        "angular_velocity": [x, y, z],
        "linear_acceleration": [x, y, z]
    },
    "temperature": 25.5,
    "battery": {
        "voltage": 12.6,
        "current": 1.5
    }
}
```

## Sorun Giderme 🔧

### GPIO Hataları

1. Pin durumlarını kontrol et:
```bash
gpio readall
```

2. I2C cihazlarını listele:
```bash
i2cdetect -y 1
```

### LIDAR Sorunları

1. USB bağlantısını kontrol et:
```bash
ls -l /dev/ttyUSB*
```

2. Port izinlerini ayarla:
```bash
sudo chmod 666 /dev/ttyUSB0
```

## Log Dosyaları 📝

- Sistem logları: `/var/log/barlas_sensor.log`
- Hata logları: `/var/log/barlas_sensor_error.log`

## Güncellemeler 🔄

Son değişiklikler:
- Ultrasonik sensör filtreleme eklendi
- IMU kalibrasyon geliştirmeleri
- LIDAR veri işleme optimizasyonu
