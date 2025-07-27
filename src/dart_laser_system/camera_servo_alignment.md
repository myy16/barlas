# 🎯 Kamera-Servo Hizalama Rehberi

## Sistem Çalışma Prensibi

**Kamera Sabit - Servo Hareket Ediyor**

```
    [KAMERA SABIT]
        |
        v  (görüş açısı)
    +----------+
    |   DART   |  ← YOLO tespit eder (piksel: 320, 240)
    +----------+
        ^
        |  (lazer ışını)
    [PAN-TILT SERVO] ← Bu açıya hareket eder
```

## ✅ DOĞRU Kurulum

```
Kamera pozisyonu: Masa/tripod üzerinde SABIT
Pan-Tilt pozisyonu: Kamera ile AYNI HİZADA veya çok yakın
Mesafe: Kamera-PanTilt arası max 10-20cm
```

## ❌ YANLIŞ Kurulum 

```
[KAMERA]     [DART]     [PAN-TILT]
   ↓           |           ↑
Bu durumda koordinat hesaplaması YANLIŞ olur!
```

## 🔧 Koordinat Dönüşümü

1. **YOLO tespit**: Dart piksel (320, 240)
2. **Kamera merkezi**: (400, 300) - 800x600 çözünürlük
3. **Fark hesabı**: (-80, -60) piksel 
4. **Açı hesabı**: -6° pan, +4.5° tilt
5. **Servo hedefi**: 84° pan, 94.5° tilt

## 🎯 Kalibrasyon

Eğer lazer tam hedefe gelmiyor:

```python
# Offset ayarı
controller.calibrate_offset(pixel_offset_x=10, pixel_offset_y=-5)
```

## 📐 FOV Hesaplama

```python
# Kamera FOV: 60° yatay, 45° dikey
horizontal_fov = 60  # ±30°
vertical_fov = 45    # ±22.5°

# Servo limitleri ile uyumlu mu?
pan_range = 170 - 10 = 160°  # Yeterli ✅
tilt_range = 150 - 30 = 120° # Yeterli ✅
```
