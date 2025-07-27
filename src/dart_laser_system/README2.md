# 🎯 BARLAS Arduino Dart System - Diğer Bilgisayarda Adım Adım Kurulum

Bu kılavuz, BARLAS Arduino Dart Targeting System'i yeni bir bilgisayarda sıfırdan kurmak için hazırlanmıştır.

## ⚠️ ÖNEMLİ: PATH (Dosya Yolu) Uyarısı

**GIT CLONE YAPTIĞIN DİZİNİ HATIRLA!** Bu kılavuzdaki tüm komutlar `barlas` klasörünün bulunduğu yere göre değişir.

**Örnek senaryolar:**
- Desktop'ta clone yaptıysan: `cd Desktop\barlas\src\dart_laser_system`
- C:\ kökünde clone yaptıysan: `cd C:\barlas\src\dart_laser_system`  
- Documents'te clone yaptıysan: `cd Documents\barlas\src\dart_laser_system`

**Her komuttan önce doğru dizinde olduğunu kontrol et:**
```bash
pwd         # Mevcut dizini göster
dir         # Dosyaları listele (arduino_pantilt.ino görülmeli)
```

## 📋 ADIM 1: Gereksinimler Kontrolü

### Sistem Gereksinimleri:
- **Windows 10/11** (Ana sistem)
- **Python 3.8+** (Test edildi: Python 3.12.6)
- **Git** (proje indirmek için)
- **Arduino IDE** (firmware yüklemek için)
- **4GB+ RAM** (8GB önerilen)

### Kontrol Komutları:
```bash
# Python 3.8+ kontrol et
python --version
# Sonuç: Python 3.8.x veya daha yeni olmalı

# Git kurulu mu kontrol et
git --version
# Sonuç: git version 2.x.x görmeli

# Pip kontrol et
pip --version
```

## 📋 ADIM 2: Projeyi İndir

```bash
# GitHub'dan projeyi klonla
git clone https://github.com/myy16/barlas.git

# Proje dizinine geç
cd barlas

# Dosya yapısını kontrol et
dir src\dart_laser_system
```

## 📋 ADIM 3: Python Kütüphanelerini Yükle

### Seçenek A: Otomatik Kurulum (Önerilen)
```powershell
# Windows PowerShell'i YÖNETICI olarak aç
# Önce proje dizinine git (git clone yaptığın yere)
cd barlas\src\dart_laser_system

# Otomatik kurulum scriptini çalıştır
PowerShell -ExecutionPolicy Bypass -File install_windows.ps1
```

### Seçenek B: Manuel Kurulum
```bash
# Önce proje dizinine git (git clone yaptığın yere)
# Dart laser system dizinine geç
cd barlas\src\dart_laser_system

# Requirements dosyasından yükle
pip install -r requirements.txt

# VEYA tek tek yükle:
pip install opencv-python>=4.8.0
pip install numpy>=1.21.0
pip install PyYAML>=6.0
pip install pyserial>=3.5

# Opsiyonel (performans için):
pip install onnxruntime
pip install matplotlib
pip install pillow
```

### Kurulum Testi:
```bash
# Python kütüphaneleri test et
python -c "import cv2, numpy, serial, yaml; print('✅ Tüm kütüphaneler yüklü!')"
```

## 📋 ADIM 4: Arduino Hazırlığı

### Arduino IDE Kurulumu:
1. **[Arduino IDE](https://www.arduino.cc/en/software)** sitesinden indir
2. **Arduino IDE'yi kur** ve çalıştır
3. **Arduino Uno'yu USB ile bağla**

### Firmware Yükleme:
1. **Arduino IDE'de:** `File > Open` 
2. **Dosya seç:** Proje dizininde `src\dart_laser_system\arduino_pantilt.ino`
3. **Board seç:** `Tools > Board > Arduino Uno`
4. **Port seç:** `Tools > Port > COM3` (veya görünen port)
5. **Upload butonuna tıkla** (➡️ ok işareti)
6. **"Yükleme tamamlandı"** mesajını bekle

### Serial Monitor Kontrolü:
1. **Tools > Serial Monitor** aç
2. **Baud rate: 9600** seç
3. **"BARLAS Arduino Pan-Tilt Ready"** mesajını gör
4. **"TEST"** yaz ve Enter'a bas
5. **"OK - Arduino Ready"** cevabını al

## 📋 ADIM 5: Donanım Bağlantıları

### Arduino Pin Bağlantı Şeması:
```
Arduino Uno Bağlantıları:
├── Pan Servo (Turuncu kablo)    → Pin 9
├── Tilt Servo (Turuncu kablo)   → Pin 10
├── Laser Diod (+)               → Pin 13
├── Servo Motor (+5V Kırmızı)    → Arduino 5V
├── Servo Motor (GND Kahverengi) → Arduino GND
├── Laser Diod (-)               → Arduino GND
└── USB Kablo                    → Bilgisayar (güç + veri)
```

### Donanım Listesi:
- **1x Arduino Uno R3**
- **2x SG90 Servo Motor** (Pan-Tilt için)
- **1x Laser Diod Modülü** (3V-5V)
- **Jumper Kablolar** (erkek-erkek)
- **Breadboard** (küçük)
- **USB Kablo** (Arduino için - veri kablosu!)

## 📋 ADIM 6: Sistem Testleri

### Test 1: Python Import Kontrolü
```bash
# Önce doğru dizinde olduğunu kontrol et
# barlas\src\dart_laser_system dizininde olmalısın
dir

# Temel kütüphaneler
python -c "import cv2; print('✅ OpenCV OK')"
python -c "import numpy; print('✅ NumPy OK')"
python -c "import serial; print('✅ PySerial OK')"
```

### Test 2: Arduino Controller Testi
```bash
# Arduino controller import testi
python -c "from arduino_controller import ArduinoPanTiltController; print('✅ Arduino Controller OK')"
```

### Test 3: YOLO Modülü Testi
```bash
# YOLO import testi
python -c "import sys; sys.path.append('..'); from dart_recognize.yolo_predictions import YOLOPredictions; print('✅ YOLO OK')"
```

### Test 4: Kamera Kontrolü
```bash
# Mevcut kameraları listele
python yolo_arduino_dart_system.py --list-cameras

# Çıktı örneği:
# 🎥 Mevcut Kameralar:
#   0: Dahili Kamera - 640x480
#   1: USB Kamera - 1920x1080
```

## 📋 ADIM 7: Ana Sistemi Başlat

### Sistem Başlatma:
```bash
# Dahili kamera ile başlat
python yolo_arduino_dart_system.py

# USB kamera ile başlat (varsa)
python yolo_arduino_dart_system.py --camera 1

# Farklı kamera ile başlat
python yolo_arduino_dart_system.py --camera 2
```

### Başarılı Başlatma Çıktısı:
```
✅ YOLO Dart Recognition modülü yüklendi!
✅ Arduino Controller modülü yüklendi!
🔍 Arduino bağlantısı deneniyor: COM3
✅ Gerçek Arduino bağlandı: COM3
✅ Kamera: 640x480
🎯 Sistem hazır!

📋 Kontroller:
  'q' - Çıkış
  'space' - Manuel laser açma/kapama
  'c' - Merkez pozisyon
  'wasd' - Manuel servo hareket
  '+/-' - YOLO güven eşiği ayarla
🎯 Kameraya DART gösterin!
```

## 🎮 ADIM 8: Sistem Kullanımı

### Klavye Kontrolleri:
| Tuş | Fonksiyon |
|-----|-----------|
| **q** | Sistemden çıkış |
| **space** | Laser manuel aç/kapat |
| **c** | Servo'ları merkeze getir |
| **w** | Tilt servo yukarı hareket |
| **s** | Tilt servo aşağı hareket |
| **a** | Pan servo sol hareket |
| **d** | Pan servo sağ hareket |
| **+** | YOLO güven eşiği artır |
| **-** | YOLO güven eşiği azalt |
| **Mouse Click** | Manuel hedefleme |

### Otomatik Dart Hedefleme Süreci:
1. **Dart'ı kameraya göster** (yeşil kutu çıkacak)
2. **Sistem 2 saniye dart'ı takip eder** (kilitlenme çubuğu)
3. **"DART KİLİTLENDİ!"** mesajı çıkar
4. **Servo'lar otomatik hedefe yönelir**
5. **Laser otomatik açılır ve hedefler**
6. **Yeni dart aramaya devam eder**

## 🔧 ADIM 9: Sorun Giderme

### Problem: "ModuleNotFoundError: No module named 'cv2'"
```bash
# Çözüm:
pip install opencv-python
```

### Problem: "ModuleNotFoundError: No module named 'serial'"
```bash
# Çözüm:
pip install pyserial
```

### Problem: "Arduino bulunamadı, simülatör kullanılıyor"
**Çözümler:**
1. **USB kablosunu kontrol et** (şarj kablosu değil, veri kablosu!)
2. **Arduino IDE'den port kontrol et**
3. **Firmware'i tekrar yükle**
4. **Driver problemi olabilir** (CH340 driver yükle)

### Problem: "Kamera açılamadı"
```bash
# Farklı kamera indeksleri dene:
python yolo_arduino_dart_system.py --camera 0  # Dahili
python yolo_arduino_dart_system.py --camera 1  # USB #1
python yolo_arduino_dart_system.py --camera 2  # USB #2
```

### Problem: "YOLO modülü bulunamadı"
**Kontrol Listesi:**
1. Git clone'un tamamlandığından emin ol
2. `dart_recognize` klasörünün var olduğunu kontrol et
3. `dart_recognize\Model\weights\best.onnx` dosyasının var olduğunu kontrol et

### Problem: Servo'lar hareket etmiyor
**Çözümler:**
1. **5V güç bağlantısını kontrol et**
2. **GND bağlantılarını kontrol et**
3. **Pin bağlantılarını kontrol et** (Pan:9, Tilt:10)
4. **Ayrı güç kaynağı kullan** (servo'lar için)

### Problem: Laser çalışmıyor
**Çözümler:**
1. **Pin 13 bağlantısını kontrol et**
2. **Laser polaritesini kontrol et** (+/- doğru bağlı mı?)
3. **Serial Monitor'da test et:** `LASER,ON` komutu gönder

### ⚠️ Problem: PATH (Dosya Yolu) Hataları
**Yaygın Path Problemleri ve Çözümleri:**

#### Problem: "No such file or directory" hatası
```bash
# Mevcut dizini kontrol et
pwd
# VEYA
cd

# Dosyaları listele
dir
# VEYA  
ls

# Doğru dizine git - GIT CLONE YAPTIĞIN YER ÖNEMLİ!
# Örnek senaryolar:

# Senaryo 1: Desktop'ta clone yaptın
cd Desktop\barlas\src\dart_laser_system

# Senaryo 2: C:\ kökünde clone yaptın
cd C:\barlas\src\dart_laser_system

# Senaryo 3: Documents'te clone yaptın
cd Documents\barlas\src\dart_laser_system

# Senaryo 4: Farklı kullanıcı dizininde
cd C:\Users\%USERNAME%\barlas\src\dart_laser_system
```

#### Problem: Arduino dosyası bulunamıyor
```bash
# Arduino IDE'de dosya açarken:
# 1. File > Open
# 2. GIT CLONE YAPTIĞIN YERE git
# 3. barlas > src > dart_laser_system > arduino_pantilt.ino seç

# Terminal'den kontrol:
dir arduino_pantilt.ino
# Bu komut "arduino_pantilt.ino" dosyası göstermeli
```

#### Problem: Python modülleri bulunamıyor
```bash
# YOLO modülü için - relative path sorunu
# Çözüm 1: Doğru dizinde olduğunu kontrol et
cd barlas\src\dart_laser_system

# Çözüm 2: Parent directory'yi kontrol et
cd ..
dir
# "dart_recognize" klasörünü görmeli

# Çözüm 3: Tam path ile test et
python -c "import sys; print(sys.path)"
```

#### Problem: Windows vs Linux Path Farkları
```bash
# Windows (Backslash kullan):
cd barlas\src\dart_laser_system
dir arduino_pantilt.ino

# Linux/Mac (Forward slash kullan):
cd barlas/src/dart_laser_system  
ls arduino_pantilt.ino
```

#### 🔧 Path Problem Çözüm Sırası:
1. **Git clone nerede yaptığını hatırla**
2. **O dizine cd komutuyla git**
3. **dir (Windows) veya ls (Linux) ile kontrol et**
4. **barlas klasörünü görmeli**
5. **cd barlas\src\dart_laser_system** (Windows)
6. **arduino_pantilt.ino ve python dosyalarını görmeli**

## 📊 ADIM 10: Başarı Kriterleri

### ✅ Sistem Çalışıyorsa Şunları Görmelisin:

1. **Kamera Görüntüsü:**
   - Canlı kamera feed'i açılır
   - 640x480 çözünürlük
   - Gerçek zamanlı görüntü

2. **YOLO Dart Tespiti:**
   - Dart gösterince yeşil kutu çıkar
   - Güven skoru görünür (0.50-0.90)
   - "DART: 0.75" gibi metin

3. **Arduino Durumu:**
   - "Arduino: Pan=90° Tilt=90°" görünür
   - "Laser: OFF/ON" durumu görünür
   - Servo pozisyonları gerçek zamanlı güncellenir

4. **Otomatik Hedefleme:**
   - 2 saniye dart takibi
   - "LOCKING 50%" progress bar
   - "LOCKED!" mesajı
   - Servo'lar hedefe yönelir
   - Laser otomatik açılır

## 🚀 ADIM 11: Hızlı Başlatma Komutu Sırası

**Her seferinde bu sırayı takip et:**

```bash
# 1. Proje klonlandığı dizine geç (örnek yollar)
# Seçenek A: Desktop'a klonladıysan
cd Desktop\barlas\src\dart_laser_system

# Seçenek B: C:\ ana dizinine klonladıysan  
cd C:\barlas\src\dart_laser_system

# Seçenek C: Documents'e klonladıysan
cd Documents\barlas\src\dart_laser_system

# Seçenek D: Mevcut dizini kontrol et
pwd
dir

# 2. Arduino'yu USB'ye tak ve bekle

# 3. Sistemi başlat
python yolo_arduino_dart_system.py

# 4. Başarı mesajlarını kontrol et:
# ✅ YOLO Dart Recognition modülü yüklendi!
# ✅ Arduino Controller modülü yüklendi!
# ✅ Gerçek Arduino bağlandı: COM3
# ✅ Kamera: 640x480
# 🎯 Sistem hazır!

# 5. Dart göster ve otomatik hedeflemeyi izle!
```

## 📞 Yardım ve Destek

### Detaylı Dokümantasyon:
- **INSTALLATION_GUIDE.md** - Kapsamlı kurulum rehberi
- **TROUBLESHOOTING.md** - Detaylı sorun giderme
- **QUICK_START.md** - 5 dakikada başlangıç
- **HARDWARE_SETUP.md** - Donanım kurulum rehberi

### Sistem Dosyaları:
- **arduino_pantilt.ino** - Arduino firmware
- **arduino_controller.py** - Python-Arduino arayüzü
- **yolo_arduino_dart_system.py** - Ana sistem dosyası
- **requirements.txt** - Python kütüphane listesi

---

## 🎯 Final Kontrol Listesi

Bu listeyi kontrol et, hepsi ✅ olmalı:

- [ ] **Python 3.8+ yüklü**
- [ ] **Git ile proje indirildi**
- [ ] **Python kütüphaneleri yüklü (cv2, numpy, serial)**
- [ ] **Arduino IDE kurulu**
- [ ] **Arduino firmware yüklendi**
- [ ] **Donanım bağlantıları yapıldı (Pan:9, Tilt:10, Laser:13)**
- [ ] **Serial Monitor'da "BARLAS Arduino Pan-Tilt Ready" görüldü**
- [ ] **Kamera tespiti başarılı**
- [ ] **YOLO modülü import ediliyor**
- [ ] **Ana sistem başlatılabiliyor**
- [ ] **Dart tespiti çalışıyor**
- [ ] **Servo'lar hareket ediyor**
- [ ] **Laser kontrol edilebiliyor**

**🎉 Hepsi ✅ ise sistem %100 hazır! Dart hedeflemesi yapmaya başlayabilirsin.**

---

**🎯 BARLAS Arduino Dart Targeting System v1.0**  
*Diğer bilgisayarda problemsiz kurulum ve çalıştırma kılavuzu*
