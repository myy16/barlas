# BARLAS ARDUINO YENİ PIN KONFIGÜRASYONU
## ✅ Güncel Gerçek Donanım Pin Haritası

### 🎮 PWM GİRİŞ PİNLERİ (Kumanda Sinyalleri)
```
Pin 2  → PWM Servo Giriş (Fren servo kontrolü için)
Pin 3  → PWM Röle Giriş (Lazer röle kontrolü için)
```

### 💡 FAR SİSTEMİ
```
Pin 6  → Far Kontrolü (HIGH/LOW)
```

### 🎯 PAN-TILT SERVO (Lazer Yönlendirme)
```
Pin 7  → Pan Servo (Yatay hareket)
Pin 8  → Tilt Servo (Dikey hareket) 
```

### 🛑 FREN SERVO SİSTEMİ
```
Pin 9  → Fren Servo 1
Pin 10 → Fren Servo 2
```

### ⚡ LAZER RÖLE KONTROLÜ
```
Pin 13 → Lazer Röle (Bizim HIGH/LOW gönderdiğimiz pin)
```

### 📊 ENCODER SİSTEMİ
```
Pin 18-19 → Encoder 1 (Sol tekerlek)
Pin 20-21 → Encoder 2 (Sağ tekerlek)
```

### 🛸 PIXHAWK KONTROL SİSTEMİ
```
Pin 23 → Pixhawk Manuel Açma/Kapama (Kumandadan gelen)
Pin 26 → Pixhawk Kontrol 1
Pin 27 → Pixhawk Kontrol 2
```

## 🎮 YENİ KOMUT LİSTESİ

### PWM Otomatik Kontrol:
- **Pin 2 PWM** → Fren servo otomatik kontrolü
- **Pin 3 PWM** → Lazer röle otomatik kontrolü

### Manuel Seri Komutlar:
```cpp
// Lazer Kontrol
lazer_on              // Pin 13 HIGH (Lazer röle aç)
lazer_off             // Pin 13 LOW (Lazer röle kapat)

// Far Kontrol  
far_on                // Pin 6 HIGH (Far aç)
far_off               // Pin 6 LOW (Far kapat)

// Fren Kontrol
fren_uygula           // Pin 9-10 fren pozisyonu
fren_birak            // Pin 9-10 nötr pozisyon

// Pan-Tilt Kontrol
MOVE,90,90            // Pan-Tilt servo hareket

// Pixhawk Kontrol
pixhawk_manuel_on     // Pin 23 HIGH (Kumanda kontrolü)
pixhawk_manuel_off    // Pin 23 LOW
pixhawk1_on           // Pin 26 HIGH
pixhawk1_off          // Pin 26 LOW
pixhawk2_on           // Pin 27 HIGH  
pixhawk2_off          // Pin 27 LOW

// Sistem Bilgileri
status                // Tüm pin durumları
encoder_read          // Encoder değerleri
pwm_status            // PWM sinyal durumları
test                  // Bağlantı testi
```

## 🔄 ÇALIŞMA MODLARı

### 1. **PWM Otomatik Modu:**
- Pin 2'den PWM → Fren servo otomatik kontrol
- Pin 3'den PWM → Lazer röle otomatik kontrol
- Interrupt tabanlı hassas ölçüm
- Noise filtreleme ve nötr bölge

### 2. **Manuel Seri Modu:**
- Seri komutlarla tüm pinleri kontrol
- Debug ve test için ideal
- Real-time durum monitoring

### 3. **Hibrit Mod:**
- PWM + Seri komut aynı anda
- Esnek kontrol imkanı
- Acil durum override

## 📈 SİSTEM ÖZELLİKLERİ

### ✅ Interrupt Sistemi:
- PWM okuma: Pin 2-3 interrupt
- Encoder okuma: Pin 18-20 interrupt
- Microsaniye hassasiyet

### ✅ Güvenlik Özellikleri:
- PWM noise filtreleme (800-2200μs)
- Nötr bölge kontrolü (85-95°)
- Safe default durumları

### ✅ Monitoring:
- Real-time PWM değerleri
- Encoder sayaçları
- Pin durumları
- Sistem health check

## 🎯 KULLANIM ÖRNEKLERİ

### Test Sekansı:
```cpp
test                  // Bağlantı kontrolü
status               // Sistem durumu
far_on               // Far testi
lazer_on             // Lazer testi
fren_uygula          // Fren testi
encoder_read         // Encoder testi
pwm_status           // PWM durumu
```

### Operasyon Sekansı:
```cpp
pixhawk_manuel_on    // Kumanda kontrolü aktif
MOVE,45,90           // Lazer yönlendirme  
lazer_on             // Hedef lazer aktif
// PWM sinyalleri otomatik fren ve lazer kontrolü yapar
```

## 🔧 TESPİT VE KALIBRASYON

### PWM Kalibrasyonu:
- **Servo PWM:** 1000-2000μs arası
- **Filtreleme:** 800-2200μs dışı ignore
- **Nötr bölge:** 85-95° arası değişiklik yok

### Fren Kalibrasyonu:
- **Offset:** -40° (Pin 9-10)
- **Nötr:** 90° (Pin 9-10)

Bu konfigürasyon artık gerçek donanımınızla tam uyumlu! 🚀
