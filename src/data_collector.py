"""
BARLAS Tabela Veri Toplama Sistemi
Kameradan görüntü toplama ve etiketleme
"""
import cv2
import numpy as np
import os
import time
import json
from datetime import datetime

class DataCollector:
    def __init__(self):
        """Veri toplama sistemini başlat"""
        print("📊 BARLAS Tabela Veri Toplama Sistemi")
        print("=" * 60)
        
        # Veri klasörleri
        self.base_path = "d:/barlas/dataset"
        self.raw_path = os.path.join(self.base_path, "raw_images")
        self.labeled_path = os.path.join(self.base_path, "labeled_data")
        
        # Kamerayı başlat
        print("1. Kamera başlatılıyor...")
        self.cap = None
        self._init_camera()
        
        # Veri sayaçları
        self.collected_counts = {
            'digit_1': 0,
            'digit_2': 0,
            'digit_3': 0,
            'digit_4': 0,
            'raw': 0
        }
        
        self._load_existing_counts()
        
    def _init_camera(self):
        """Kamerayı başlat"""
        # DirectShow backend ile kamerayı aç
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0, cv2.CAP_MSMF)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(0)
                if not self.cap.isOpened():
                    print("   ❌ HATA: Kamera açılamadı!")
                    return False
        
        # Kamera ayarları
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        print("   ✅ Kamera başarıyla açıldı")
        return True
    
    def _load_existing_counts(self):
        """Mevcut veri sayılarını yükle"""
        for digit_folder in ['digit_1', 'digit_2', 'digit_3', 'digit_4']:
            folder_path = os.path.join(self.labeled_path, digit_folder)
            if os.path.exists(folder_path):
                count = len([f for f in os.listdir(folder_path) if f.endswith('.jpg')])
                self.collected_counts[digit_folder] = count
        
        # Raw images
        if os.path.exists(self.raw_path):
            count = len([f for f in os.listdir(self.raw_path) if f.endswith('.jpg')])
            self.collected_counts['raw'] = count
    
    def run_collection(self):
        """Ana veri toplama döngüsü"""
        if self.cap is None or not self.cap.isOpened():
            print("❌ Kamera başlatılamadı!")
            return
        
        print("\n2. Veri toplama modu başlıyor...")
        print("   📋 Kontroller:")
        print("   - 1: Rakam 1 olarak etiketle ve kaydet")
        print("   - 2: Rakam 2 olarak etiketle ve kaydet") 
        print("   - 3: Rakam 3 olarak etiketle ve kaydet")
        print("   - 4: Rakam 4 olarak etiketle ve kaydet")
        print("   - R: Ham görüntü olarak kaydet (etiketlenmemiş)")
        print("   - S: Anlık istatistikleri göster")
        print("   - Q: Çıkış")
        print("\n   🎯 Hedef: Her rakam için en az 200 örnek")
        print("=" * 60)
        
        frame_count = 0
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("❌ Frame okunamadı!")
                break
            
            frame_count += 1
            
            # Display frame hazırla
            display_frame = frame.copy()
            
            # Bilgi yazıları
            cv2.putText(display_frame, "BARLAS - Veri Toplama", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Veri sayıları
            y_pos = 70
            cv2.putText(display_frame, f"Digit 1: {self.collected_counts['digit_1']}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_pos += 25
            cv2.putText(display_frame, f"Digit 2: {self.collected_counts['digit_2']}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_pos += 25
            cv2.putText(display_frame, f"Digit 3: {self.collected_counts['digit_3']}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_pos += 25
            cv2.putText(display_frame, f"Digit 4: {self.collected_counts['digit_4']}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_pos += 25
            cv2.putText(display_frame, f"Raw: {self.collected_counts['raw']}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Kontrol yazıları
            cv2.putText(display_frame, "1-4: Etiketle | R: Ham kaydet | S: Stats | Q: Cikis", 
                       (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Çerçeve göster
            cv2.imshow('BARLAS - Veri Toplama', display_frame)
            
            # Klavye kontrolleri
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # Çıkış
                print("\n🛑 Veri toplama kapatılıyor...")
                break
            elif key == ord('1'):  # Digit 1
                self._save_labeled_image(frame, 'digit_1')
            elif key == ord('2'):  # Digit 2
                self._save_labeled_image(frame, 'digit_2')
            elif key == ord('3'):  # Digit 3
                self._save_labeled_image(frame, 'digit_3')
            elif key == ord('4'):  # Digit 4
                self._save_labeled_image(frame, 'digit_4')
            elif key == ord('r'):  # Raw image
                self._save_raw_image(frame)
            elif key == ord('s'):  # Statistics
                self._show_statistics()
        
        # Temizlik
        self.cap.release()
        cv2.destroyAllWindows()
        print("✅ Veri toplama sistemi kapatıldı")
        self._save_final_statistics()
    
    def _save_labeled_image(self, frame, label):
        """Etiketli görüntü kaydet"""
        timestamp = int(time.time() * 1000)
        filename = f"{label}_{timestamp}.jpg"
        folder_path = os.path.join(self.labeled_path, label)
        
        # Klasör yoksa oluştur
        os.makedirs(folder_path, exist_ok=True)
        
        filepath = os.path.join(folder_path, filename)
        
        # Görüntüyü kaydet
        success = cv2.imwrite(filepath, frame)
        
        if success:
            self.collected_counts[label] += 1
            digit_num = label.split('_')[1]
            print(f"✅ Rakam {digit_num} kaydedildi: {filename} (Toplam: {self.collected_counts[label]})")
        else:
            print(f"❌ Kaydetme hatası: {filename}")
    
    def _save_raw_image(self, frame):
        """Ham görüntü kaydet"""
        timestamp = int(time.time() * 1000)
        filename = f"raw_{timestamp}.jpg"
        
        # Klasör yoksa oluştur
        os.makedirs(self.raw_path, exist_ok=True)
        
        filepath = os.path.join(self.raw_path, filename)
        
        # Görüntüyü kaydet
        success = cv2.imwrite(filepath, frame)
        
        if success:
            self.collected_counts['raw'] += 1
            print(f"📷 Ham görüntü kaydedildi: {filename} (Toplam: {self.collected_counts['raw']})")
        else:
            print(f"❌ Kaydetme hatası: {filename}")
    
    def _show_statistics(self):
        """Veri istatistiklerini göster"""
        print("\n" + "=" * 40)
        print("📊 VERİ İSTATİSTİKLERİ")
        print("=" * 40)
        
        total_labeled = sum([self.collected_counts[f'digit_{i}'] for i in range(1, 5)])
        
        for i in range(1, 5):
            count = self.collected_counts[f'digit_{i}']
            progress = min(100, (count / 200) * 100)
            print(f"Rakam {i}: {count:4d} örnek ({progress:5.1f}%)")
        
        print(f"Ham resim: {self.collected_counts['raw']:4d} örnek")
        print(f"Toplam etiketli: {total_labeled:4d} örnek")
        print("=" * 40)
        
        # Tavsiyeler
        if total_labeled < 800:
            needed = 800 - total_labeled
            print(f"🎯 Hedef için {needed} örnek daha gerekli")
        else:
            print("🎉 Hedef veri miktarına ulaşıldı!")
        
        print()
    
    def _save_final_statistics(self):
        """Final istatistikleri JSON olarak kaydet"""
        stats = {
            'collection_date': datetime.now().isoformat(),
            'counts': self.collected_counts,
            'total_labeled': sum([self.collected_counts[f'digit_{i}'] for i in range(1, 5)]),
            'total_raw': self.collected_counts['raw']
        }
        
        stats_file = os.path.join(self.base_path, 'collection_stats.json')
        with open(stats_file, 'w') as f:
            json.dump(stats, f, indent=2)
        
        print(f"📊 İstatistikler kaydedildi: {stats_file}")

def main():
    """Ana fonksiyon"""
    try:
        collector = DataCollector()
        collector.run_collection()
    except KeyboardInterrupt:
        print("\n🛑 Kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"❌ Sistem hatası: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
