"""
Test sistemi - digit_recognizer.py ve tabela_recognition.py kontrol
"""
import sys
import os
sys.path.append(r'd:\barlas\src')

try:
    print("1. digit_recognizer.py import ediliyor...")
    from digit_recognizer import DigitRecognizer
    print("   ✅ digit_recognizer başarıyla import edildi")
    
    print("2. DigitRecognizer sınıfı oluşturuluyor...")
    recognizer = DigitRecognizer()
    print("   ✅ DigitRecognizer başarıyla oluşturuldu")
    
    print("3. tabela_recognition.py import ediliyor...")
    from tabela_recognition import TabelaRecognition
    print("   ✅ tabela_recognition başarıyla import edildi")
    
    print("4. TabelaRecognition sınıfı oluşturuluyor...")
    tabela_rec = TabelaRecognition()
    print("   ✅ TabelaRecognition başarıyla oluşturuldu")
    
    print("5. OpenCV test görüntüsü oluşturuluyor...")
    import cv2
    import numpy as np
    
    # Test görüntüsü oluştur - beyaz zemin, siyah rakam 1
    test_img = np.ones((200, 200, 3), dtype=np.uint8) * 255
    cv2.putText(test_img, '1', (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 0), 8)
    print("   ✅ Test görüntüsü oluşturuldu")
    
    print("6. analyze_image metodunu test ediliyor...")
    result = tabela_rec.analyze_image(test_img, use_cnn=False)
    print(f"   ✅ Sonuç: {result}")
    
    print("\n🎉 TÜM TESTLER BAŞARILI!")
    
except ImportError as e:
    print(f"❌ Import hatası: {e}")
except Exception as e:
    print(f"❌ Genel hata: {e}")
    import traceback
    traceback.print_exc()
