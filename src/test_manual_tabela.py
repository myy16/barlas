"""
Manuel tabela test scripti - farklı renklerde tabelalar oluşturur ve test eder
"""
import cv2
import numpy as np
from tabela_recognition import TabelaRecognition

def create_test_tabela(color, etap_name):
    """Test tabela görüntüsü oluşturur"""
    # 640x480 boyutunda görüntü
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Arka plan (gri)
    img.fill(50)
    
    # Merkeze 60 cm çapında beyaz daire (tabela arka planı)
    cv2.circle(img, (320, 240), 150, (255, 255, 255), -1)
    
    # İçine renkli daire (etap göstergesi)
    cv2.circle(img, (320, 240), 80, color, -1)
    
    # Etap ismini yaz
    cv2.putText(img, etap_name, (250, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return img

def test_manual_tabela():
    print("=== Manuel Tabela Test ===")
    
    # Tabela recognition başlat
    tabela_rec = TabelaRecognition()
    
    # Test etapları ve renkleri
    test_etaplar = [
        ((0, 255, 0), "duz_yol"),           # Yeşil
        ((0, 0, 255), "dik_egim"),          # Kırmızı
        ((255, 255, 0), "hizlanma"),        # Sarı
        ((255, 0, 255), "sigh_su"),         # Magenta
        ((0, 255, 255), "yan_egim"),        # Cyan
        ((128, 128, 128), "tasli"),         # Gri
        ((255, 165, 0), "trafik_konileri"), # Turuncu
        ((128, 0, 128), "engebeli")         # Mor
    ]
    
    for color, etap_name in test_etaplar:
        print(f"\n--- {etap_name.upper()} TEST ---")
        
        # Test görüntüsü oluştur
        test_img = create_test_tabela(color, etap_name)
        
        # Tabela tanıma yap
        etap, confidence = tabela_rec.recognize_from_frame(test_img)
        
        print(f"Beklenen: {etap_name} → Bulunan: {etap} (güven: {confidence:.2f})")
        
        # Görüntüyü göster (2 saniye)
        cv2.imshow(f'Test: {etap_name}', test_img)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_manual_tabela()
