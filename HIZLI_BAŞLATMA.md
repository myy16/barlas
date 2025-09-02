# ✅ BARLAS SİSTEMİ - HIZLI KONTROL LİSTESİ

Arkadaşınızın sistemi çalıştırması için bu adımları takip etsin:

## 🔥 HIZLI BAŞLATMA (5 DAKİKA)

### 1. Arduino Hazırlığı (2dk)
```
✅ Arduino'yu USB'ye bağla
✅ Arduino IDE aç → arduino_pantilt_fixed.ino dosyasını yükle
✅ Serial Monitor'de "TEST" yazıp "OK" cevabını al
```

### 2. Python Kütüphaneleri (2dk)
```bash
pip install opencv-python ultralytics pyserial numpy Pillow
```

### 3. Sistemi Çalıştır (1dk)
```bash
# İlk çalıştırma - Test
cd d:\barlas\src\dart_laser_system
python test_system_integration.py

# Ana sistem
python yolo_arduino_dart_system.py
```

## 🎮 KULLANIM KONTROLLARI

| Tuş | Fonksiyon |
|-----|-----------|
| `q` | Çıkış |
| `l` | Lazer açma/kapama |
| `c` | Merkez pozisyonu |
| `r` | Reset |
| `w/s/a/d` | Manuel hareket |
| **Sol Tık** | Manuel hedefleme |

## 🔧 HATA ÇÖZÜMÜ

### Arduino Bağlanamıyor?
```python
# Port kontrolü
python -c "import serial.tools.list_ports; [print(p) for p in serial.tools.list_ports.comports()]"
```

### Kamera Açılmıyor?
```python
# Kamera testi
python -c "import cv2; cap = cv2.VideoCapture(0); print('Kamera OK' if cap.isOpened() else 'HATA'); cap.release()"
```

### Model Bulunamıyor?
```
✅ best.onnx dosyasının d:\barlas\src\dart_laser_system\ klasöründe olduğunu kontrol et
```

## 📞 ACIL DURUM

**Sistem donuyorsa:** `Ctrl+C` veya pencereyi kapat
**Arduino yanıt vermiyorsa:** USB kablosunu çıkar-tak
**Kamera donuyorsa:** Başka kamera indeksi dene (0, 1, 2...)

## 🎯 BAŞARI GÖSTERGELERİ

✅ Arduino: "BARLAS Arduino Ready - Full System" mesajı
✅ Python: "YOLO model yüklendi" mesajı  
✅ Kamera: Görüntü penceresi açıldı
✅ Dart tespit: Yeşil kutucuklar görünür

**Bu 4 şey tamsa sistem hazır! 🚀**

---

💡 **PRO İPUCU:** İlk çalıştırmadan önce mutlaka `python test_system_integration.py` yap!
