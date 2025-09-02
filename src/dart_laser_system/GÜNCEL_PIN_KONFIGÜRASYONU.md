# BARLAS GÜNCEL PIN KONFİGÜRASYONU
# Arduino Mega 2560 - Güncellenmiş Pin Atamaları
# Tarih: 3 Eylül 2025

## 🎯 KONTROL PİNLERİ

### PWM Giriş Sinyalleri
- **Pin 2**: PWM Servo Girişi (Fren servo kontrol sinyali)
- **Pin 3**: PWM Röle Girişi (Lazer röle kontrol sinyali)

### Far Sistemi
- **Pin 6**: Far LED/Röle Kontrol

### Pan-Tilt Servo Sistemi (X-Y Ekseni)
- **Pin 7**: Pan Servo (X ekseni - sağ/sol)
- **Pin 8**: Tilt Servo (Y ekseni - yukarı/aşağı)

### Fren Servo Sistemi
- **Pin 9**: Fren Servo 1
- **Pin 10**: Fren Servo 2

### Lazer Sistemi
- **Pin 13**: Lazer Röle Kontrolü

### Encoder Sistemi
- **Pin 18**: Sol Encoder A Kanalı (Interrupt 5)
- **Pin 19**: Sol Encoder B Kanalı (Interrupt 4) 
- **Pin 20**: Sağ Encoder A Kanalı (Interrupt 3)
- **Pin 21**: Sağ Encoder B Kanalı (Interrupt 2)

### Pixhawk Kontrol Sistemi
- **Pin 23**: Pixhawk Manuel Açma/Kapama
- **Pin 26**: Pixhawk X Ekseni Kontrol
- **Pin 27**: Pixhawk Y Ekseni Kontrol

---

## 🔧 SİSTEM KOMUTLARI

### Pan-Tilt Kontrol
```
MOVE,45,60        # Pan=45°, Tilt=60° pozisyona git
```

### Lazer Kontrol
```
LASER,ON          # Lazer röle aç (Pin 13 HIGH)
LASER,OFF         # Lazer röle kapat (Pin 13 LOW)
lazer_on          # Alternatif komut
lazer_off         # Alternatif komut
```

### Far Kontrol
```
far_on            # Far aç (Pin 6 HIGH)
far_off           # Far kapat (Pin 6 LOW)
```

### Fren Servo Kontrol
```
fren_uygula       # Fren servo açılarını uygula
fren_birak        # Fren servo nötr pozisyon
```

### Encoder Sistemi
```
encoder_read      # Encoder değerlerini oku
encoder_reset     # Encoder sayaçlarını sıfırla
```

### Pixhawk Kontrol
```
pixhawk_manuel_on    # Pin 23 HIGH
pixhawk_manuel_off   # Pin 23 LOW
pixhawk1_on         # Pin 26 HIGH (X ekseni)
pixhawk1_off        # Pin 26 LOW
pixhawk2_on         # Pin 27 HIGH (Y ekseni)  
pixhawk2_off        # Pin 27 LOW
```

### Sistem Bilgi
```
status            # Tüm pin durumları ve sensor değerleri
test              # Bağlantı testi
pwm_status        # PWM sinyal durumları
```

---

## 📊 ÇIKIŞ FORMATLARI

### Encoder Okuma Çıktısı
```
OK - Encoders L:1234,R:5678
```

### Status Çıktısı
```
=== BARLAS SİSTEM DURUM (GÜNCEL PIN) ===
Encoder1 (Pin18-19): 1234
Encoder2 (Pin20-21): 5678
Servo PWM (Pin2): 1500μs
Lazer PWM (Pin3): 1000μs
Far (Pin6): OFF
Pan-Tilt (Pin7-8): Pan=90° Tilt=90°
Fren (Pin9-10): 90° / 90°
Lazer Röle (Pin13): OFF
Pixhawk Manuel (Pin23): OFF
Pixhawk X-Y (Pin26-27): OFF / OFF
========================================
```

---

## ⚡ INTERRUPT SİSTEMİ

### PWM Interrupt (Pin 2-3)
- Rising edge ve falling edge interrupt aktif
- Pulse width ölçümü sürekli yapılıyor
- Fren servo ve lazer röle otomatik kontrolü

### Encoder Interrupt (Pin 18-21)  
- Quadrature encoder okuma
- A kanalı interrupt (CHANGE tetikleme)
- B kanalı direction belirleme için okunuyor
- Sayaç artırma/azaltma otomatik

---

## 🔌 DONANIM BAĞLANTI REHBERİ

```
Arduino Mega 2560
├── Pin 2  ← PWM Servo Kumanda Sinyali
├── Pin 3  ← PWM Lazer Kumanda Sinyali  
├── Pin 6  → Far LED/Röle
├── Pin 7  → Pan Servo (X)
├── Pin 8  → Tilt Servo (Y)
├── Pin 9  → Fren Servo 1
├── Pin 10 → Fren Servo 2
├── Pin 13 → Lazer Röle
├── Pin 18 ← Sol Encoder A
├── Pin 19 ← Sol Encoder B
├── Pin 20 ← Sağ Encoder A
├── Pin 21 ← Sağ Encoder B
├── Pin 23 → Pixhawk Manuel
├── Pin 26 → Pixhawk X Kontrol
└── Pin 27 → Pixhawk Y Kontrol
```

**Not**: Bu konfigürasyon ROS encoder node ile uyumlu şekilde güncellenmiştir.
