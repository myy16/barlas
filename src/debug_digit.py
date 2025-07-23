"""
Digit Recognizer Debug Test
"""
import sys
sys.path.append(r'd:\barlas\src')

import cv2
import numpy as np
from digit_recognizer import DigitRecognizer

def create_test_image(digit):
    """Test görüntüsü oluştur"""
    img = np.ones((200, 200, 3), dtype=np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 8
    thickness = 10
    
    text_size = cv2.getTextSize(str(digit), font, font_scale, thickness)[0]
    x = (img.shape[1] - text_size[0]) // 2
    y = (img.shape[0] + text_size[1]) // 2
    
    cv2.putText(img, str(digit), (x, y), font, font_scale, (0, 0, 0), thickness)
    return img

def debug_digit_recognizer():
    print("🔍 Digit Recognizer Debug Test")
    print("=" * 50)
    
    recognizer = DigitRecognizer()
    
    for digit in [1, 2, 3, 4]:
        print(f"\n🔢 Rakam {digit} debug:")
        
        # Test görüntüsü oluştur
        img = create_test_image(digit)
        print(f"   📸 Görüntü boyutu: {img.shape}")
        
        # Gri tonlama
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        print(f"   🔄 Gri tonlama: {gray.shape}")
        
        # Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        print(f"   🔄 Blur uygulandı")
        
        # Adaptive threshold
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY_INV, 11, 2)
        print(f"   🔄 Threshold uygulandı")
        
        # Contour bulma
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"   📐 Bulunan contour sayısı: {len(contours)}")
        
        # Contour'ları değerlendir
        valid_contours = 0
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            print(f"      Contour {i}: alan = {area}")
            
            if area < 500:
                print(f"         ❌ Çok küçük alan")
                continue
                
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = h / w
            print(f"         📏 Boyutlar: {w}x{h}, aspect_ratio: {aspect_ratio:.2f}")
            
            if aspect_ratio < 1.2 or aspect_ratio > 3.0:
                print(f"         ❌ Aspect ratio uygun değil")
                continue
                
            valid_contours += 1
            print(f"         ✅ Geçerli contour")
        
        print(f"   📊 Geçerli contour sayısı: {valid_contours}")
        
        # Tanıma testi
        digit_result, confidence = recognizer.recognize_digit_in_frame(img)
        print(f"   🎯 Tanıma sonucu: {digit_result}, güven: {confidence:.3f}")
        
        # Test görüntülerini kaydet
        cv2.imwrite(f'd:/barlas/debug_digit_{digit}_original.jpg', img)
        cv2.imwrite(f'd:/barlas/debug_digit_{digit}_threshold.jpg', thresh)
        print(f"   💾 Debug görüntüleri kaydedildi")

if __name__ == "__main__":
    debug_digit_recognizer()
