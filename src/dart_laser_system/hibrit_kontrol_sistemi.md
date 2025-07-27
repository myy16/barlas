# 🎯 BARLAS Hibrit Kontrol Sistemi

## 🔄 Otomatik + Manuel Kontrol

```
📡 SİSTEM DURUMU: SÜREKLE ÇALIŞAN
├── [OTOMATIK] YOLO Dart Detection  (background)
├── [OTOMATIK] Hough Circle Merkez  (background)  
├── [OTOMATIK] 2sn Kilitlenme Logic (background)
└── [MANUEL] Klavye Input Listener   (foreground)
```

## 🎮 Kontrol Modları

### 🤖 OTOMATIK MOD
```python
# Sürekli çalışan otomatik sistem
while True:
    # YOLO dart tespit ediyor
    dart_detections = yolo.detect(frame)
    
    if dart_found:
        # 2 saniye bekle
        if lock_time >= 2.0:
            # OTOMATİK servo hareketi
            arduino.aim_at_pixel(dart_center)
            arduino.enable_laser()
    
    # MANUEL kontrol dinle
    key = cv2.waitKey(1)
    if key == ord('w'):  # MANUEL müdahale
        arduino.move_tilt_up()
```

### 🎮 MANUEL OVERRIDE
```python
# Manuel kontrol HER ZAMAN aktif
if key == ord('w'):      # Yukarı
    arduino.tilt_up()    # Otomatik sistemin üzerinde çalışır
elif key == ord(' '):   # Space
    arduino.toggle_laser()  # Otomatik lazer'i manuel kontrol
elif key == ord('c'):   # Center  
    arduino.center()     # Manuel pozisyon sıfırlama
```

## ⚙️ Öncelik Sistemi

```
1. MANUEL KOMUT ALINDI MI?
   ├── EVET → Manuel komutu HEMEN çalıştır
   └── HAYIR → Otomatik sistemin devam etmesine izin ver

2. OTOMATIK HEDEF VAR MI?
   ├── EVET → 2 saniye bekle → Otomatik servo hareketi  
   └── HAYIR → Sadece manuel kontrol aktif
```

## 🎯 Gerçek Zamanlı Örnek

```
⏰ T=0s:   [MANUEL] 'w' tuşu → Tilt yukarı hareket
⏰ T=1s:   [OTOMATIK] Dart tespit edildi, kilitlenme başladı
⏰ T=1.5s: [MANUEL] 'space' tuşu → Lazer manuel açıldı
⏰ T=3s:   [OTOMATIK] 2sn tamamlandı → Servo otomatik hareketi
⏰ T=3.5s: [MANUEL] 'a' tuşu → Pan sol hareket (otomatik hareketin üzerine)
```

## 🔧 Kontrol Öncelikleri

### YÜKSEK ÖNCELİK (Manuel)
- **WASD**: Anında servo hareketi
- **Space**: Anında lazer toggle
- **C**: Anında merkez pozisyon
- **Q**: Anında çıkış

### DÜŞÜK ÖNCELİK (Otomatik) 
- **YOLO Detection**: Background'da çalışır
- **2sn Kilitlenme**: Manuel komut yoksa çalışır
- **Otomatik Servo**: Manuel hareket yoksa çalışır

## 🎮 Kullanım Senaryoları

### Senaryo 1: Tam Otomatik
```
1. Sistem başlat
2. Dart göster
3. 2 saniye bekle
4. ✅ Otomatik hedefleme ve lazer
```

### Senaryo 2: Manuel Müdahale
```
1. Sistem başlat
2. Dart göster  
3. YOLO tespit etti ama sen WASD ile kendin ayarla
4. Space ile lazeri manuel kontrol et
```

### Senaryo 3: Hibrit Kullanım
```
1. Otomatik tespit ve kaba ayar
2. Manuel ince ayar (WASD)
3. Manuel lazer kontrolü (Space)
```

## 🔄 Kod Akışı

```python
def run_targeting_system():
    while True:
        # 1. OTOMATIK SİSTEM
        frame = camera.read()
        darts = yolo.detect(frame)
        
        if dart_locked_2_seconds:
            arduino.auto_aim(dart_center)  # Otomatik
        
        # 2. MANUEL SİSTEM (Paralel)
        key = keyboard.read()
        
        if key == 'w':
            arduino.manual_tilt_up()       # Manuel override
        elif key == 'space':
            arduino.manual_laser_toggle()  # Manuel override
        
        # 3. Her iki sistem de aktif!
```

## 🎯 Avantajları

✅ **Tam Otomatik**: Dart göster, sistem halleder  
✅ **Manuel Override**: İstediğin zaman müdahale et  
✅ **Hibrit Kullanım**: Otomatik + Manuel kombinasyon  
✅ **Gerçek Zamanlı**: Anında tepki  
✅ **Kullanıcı Dostu**: Öğrenme kolaylığı  

## 🎮 Pratik Kullanım

```bash
# Sistem başlat
python yolo_arduino_dart_system.py --camera 1

# Göreceksin:
📹 Kamera açıldı
🤖 Arduino bağlandı  
🎯 YOLO hazır
🎮 Kontroller aktif!

# Artık hem otomatik çalışır, hem sen kontrol edersin!
```
