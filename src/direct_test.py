"""
Basit Test Script - Web interface olmadan direct test
"""
import sys
sys.path.append(r'd:\barlas\src')

import cv2
import numpy as np
import base64
from tabela_recognition import TabelaRecognition

def create_test_image(digit):
    """Test görüntüsü oluştur"""
    # 200x200 beyaz background
    img = np.ones((200, 200, 3), dtype=np.uint8) * 255
    
    # Siyah rakam çiz
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 8
    thickness = 10
    
    # Merkeze yerleştir
    text_size = cv2.getTextSize(str(digit), font, font_scale, thickness)[0]
    x = (img.shape[1] - text_size[0]) // 2
    y = (img.shape[0] + text_size[1]) // 2
    
    cv2.putText(img, str(digit), (x, y), font, font_scale, (0, 0, 0), thickness)
    return img

def test_direct():
    """Web interface olmadan direct test"""
    print("🚗 BARLAS - Direct Tabela Test")
    print("=" * 50)
    
    # Tabela tanıma sistemini başlat
    print("1. TabelaRecognition sistemi başlatılıyor...")
    tabela_recognition = TabelaRecognition()
    print("   ✅ Başlatıldı")
    
    # Her rakam için test
    for digit in [1, 2, 3, 4]:
        print(f"\n2.{digit} Rakam {digit} testi:")
        
        # Test görüntüsü oluştur
        img = create_test_image(digit)
        print(f"   📸 Test görüntüsü oluşturuldu: {img.shape}")
        
        # Base64 encode/decode test (web'de yapılan işlem)
        _, buffer = cv2.imencode('.jpg', img)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        print(f"   📦 Base64 encode: {len(img_base64)} karakter")
        
        # Base64'ten geri decode
        img_data = base64.b64decode(img_base64)
        img_decoded = cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)
        print(f"   📦 Base64 decode: {img_decoded.shape}")
        
        # Analiz et
        print(f"   🔍 Analiz başlatılıyor...")
        try:
            result = tabela_recognition.analyze_image(img_decoded, use_cnn=False)
            print(f"   ✅ Sonuç: {result}")
            
            if result['etap'] != 'düz':
                print(f"   🎯 BAŞARILI! Rakam {digit} -> {result['etap']}")
            else:
                print(f"   ⚠️  Rakam tanınamadı, varsayılan döndürüldü")
                
        except Exception as e:
            print(f"   ❌ HATA: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "=" * 50)
    print("Test tamamlandı!")

if __name__ == "__main__":
    test_direct()
