"""
Gelişmiş Rakam Tanıma Sistemi
OpenCV kullanarak contour ve digit recognition
Multiple template ve threshold yöntemleri
"""
import cv2
import numpy as np

class DigitRecognizer:
    def __init__(self):
        """
        Rakam tanıma sistemini başlat
        """
        self.digit_templates = {}
        self._create_digit_templates()
    
    def _create_digit_templates(self):
        """
        1, 2, 3, 4 rakamları için gelişmiş template'ler oluştur
        Her rakam için multiple variation'lar
        """
        self.digit_templates = {}
        
        # Her rakam için birden fazla template oluştur
        for digit in [1, 2, 3, 4]:
            self.digit_templates[digit] = []
            
            # Template 1: Kalın font
            template1 = self._create_font_template(digit, thick=True)
            self.digit_templates[digit].append(template1)
            
            # Template 2: İnce font
            template2 = self._create_font_template(digit, thick=False)
            self.digit_templates[digit].append(template2)
            
            # Template 3: Segment style (7-segment display gibi)
            template3 = self._create_segment_template(digit)
            self.digit_templates[digit].append(template3)
    
    def _create_font_template(self, digit, thick=True):
        """Font tabanlı template oluştur"""
        template = np.zeros((60, 40), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 2.5
        thickness = 4 if thick else 2
        
        # Merkeze yerleştir
        text_size = cv2.getTextSize(str(digit), font, font_scale, thickness)[0]
        x = (40 - text_size[0]) // 2
        y = (60 + text_size[1]) // 2
        
        cv2.putText(template, str(digit), (x, y), font, font_scale, 255, thickness)
        return template
    
    def _create_segment_template(self, digit):
        """7-segment display style template"""
        template = np.zeros((60, 40), dtype=np.uint8)
        
        # 7-segment pozisyonları
        segments = {
            1: ['b', 'c'],  # Sadece sağ dikey çizgiler
            2: ['a', 'b', 'g', 'e', 'd'],  # 2 için gerekli segmentler
            3: ['a', 'b', 'g', 'c', 'd'],  # 3 için gerekli segmentler  
            4: ['f', 'g', 'b', 'c']  # 4 için gerekli segmentler
        }
        
        # Segment koordinatları
        seg_coords = {
            'a': [(8, 5), (32, 10)],    # üst yatay
            'b': [(32, 5), (37, 27)],   # sağ üst dikey
            'c': [(32, 27), (37, 50)],  # sağ alt dikey  
            'd': [(8, 50), (32, 55)],   # alt yatay
            'e': [(3, 27), (8, 50)],    # sol alt dikey
            'f': [(3, 5), (8, 27)],     # sol üst dikey
            'g': [(8, 25), (32, 30)]    # orta yatay
        }
        
        # İlgili segmentleri çiz
        for seg in segments[digit]:
            x1, y1 = seg_coords[seg][0]
            x2, y2 = seg_coords[seg][1]
            cv2.rectangle(template, (x1, y1), (x2, y2), 255, -1)
            
        return template

    def recognize_digit_in_frame(self, frame):
        """
        Frame içindeki rakamları tanı - gelişmiş yöntem
        """
        try:
            # Görüntü ön işleme - daha gelişmiş
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Histogram equalization ile kontrastı artır
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            gray = clahe.apply(gray)
            
            # Gaussian blur uygula
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Multiple threshold method dene
            thresholds = []
            
            # Method 1: Adaptive threshold
            thresh1 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY_INV, 11, 2)
            thresholds.append(("Adaptive", thresh1))
            
            # Method 2: Otsu threshold  
            _, thresh2 = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            thresholds.append(("Otsu", thresh2))
            
            # Method 3: Fixed threshold
            _, thresh3 = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)
            thresholds.append(("Fixed", thresh3))
            
            best_result = None
            best_confidence = 0.0
            
            # Her threshold yöntemi için contour analizi yap
            for method_name, thresh in thresholds:
                print(f"[DEBUG] {method_name} threshold deneniyor...")
                
                # Morphological operations ile gürültüyü temizle
                kernel = np.ones((3,3), np.uint8)
                thresh_clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
                thresh_clean = cv2.morphologyEx(thresh_clean, cv2.MORPH_OPEN, kernel)
                
                result = self._analyze_contours(thresh_clean)
                if result and result[1] > best_confidence:
                    best_result = result
                    best_confidence = result[1]
                    print(f"[DEBUG] {method_name} yöntemi ile daha iyi sonuç: {best_confidence:.3f}")
            
            if best_result:
                return best_result
            else:
                print(f"[DEBUG] Hiçbir threshold method'u sonuç vermedi")
                return None, 0.0
                
        except Exception as e:
            print(f"Rakam tanıma hatası: {e}")
            return None, 0.0
    
    def _analyze_contours(self, thresh):
        """Contour analizi yapar"""
        # Contour'ları bul
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_digit = None
        best_confidence = 0.0
        best_area = 0
        
        print(f"[DEBUG] Bulunan contour sayısı: {len(contours)}")
        
        # Her contour'u değerlendir
        for i, contour in enumerate(contours):
            # Çok küçük alanları filtrele
            area = cv2.contourArea(contour)
            if area < 100:  # Daha düşük minimum
                print(f"[DEBUG] Contour {i}: alan çok küçük ({area})")
                continue
            
            # Bounding box al
            x, y, w, h = cv2.boundingRect(contour)
            
            # Aspect ratio kontrol - daha gevşek
            aspect_ratio = h / w
            if aspect_ratio < 0.8 or aspect_ratio > 5.0:  # Daha geniş aralık
                print(f"[DEBUG] Contour {i}: aspect ratio uygun değil ({aspect_ratio:.2f})")
                continue
            
            # Minimum boyut kontrolü
            if w < 10 or h < 15:
                print(f"[DEBUG] Contour {i}: boyut çok küçük ({w}x{h})")
                continue
                
            print(f"[DEBUG] Contour {i}: geçerli - alan={area}, boyut={w}x{h}, aspect={aspect_ratio:.2f}")
            
            # ROI'yi çıkar ve standart boyuta getir
            roi = thresh[y:y+h, x:x+w]
            roi_resized = cv2.resize(roi, (40, 60))  # Template boyutuna getir
            
            # Her digit'in tüm template'leri ile karşılaştır
            best_match_score = 0
            best_match_digit = None
            
            for digit, templates in self.digit_templates.items():
                best_digit_score = 0
                
                # Bu digit'in tüm template'lerini dene
                for template_idx, template in enumerate(templates):
                    # Multiple matching yöntemleri dene
                    methods = [
                        ("CCORR_NORMED", cv2.TM_CCORR_NORMED),
                        ("CCOEFF_NORMED", cv2.TM_CCOEFF_NORMED),
                        ("SQDIFF_NORMED", cv2.TM_SQDIFF_NORMED)
                    ]
                    
                    for method_name, method in methods:
                        result = cv2.matchTemplate(roi_resized, template, method)
                        
                        if method == cv2.TM_SQDIFF_NORMED:
                            # SQDIFF için düşük değer daha iyi
                            min_val, _, _, _ = cv2.minMaxLoc(result)
                            score = 1.0 - min_val  # Tersine çevir
                        else:
                            # Diğerleri için yüksek değer daha iyi
                            _, max_val, _, _ = cv2.minMaxLoc(result)
                            score = max_val
                        
                        # Bu digit için en iyi template skorunu bul
                        if score > best_digit_score:
                            best_digit_score = score
                
                print(f"[DEBUG] Digit {digit} best score: {best_digit_score:.3f}")
                
                # En iyi digit skorunu bul
                if best_digit_score > best_match_score:
                    best_match_score = best_digit_score
                    best_match_digit = digit
            
            # Alan ağırlığı ekle - daha dengeli
            area_weight = min(1.0, area / 1000)  # Normalize et
            weighted_confidence = best_match_score * (0.7 + 0.3 * area_weight)
            
            print(f"[DEBUG] En iyi match: digit {best_match_digit}, score: {best_match_score:.3f}, weighted: {weighted_confidence:.3f}")
            
            if weighted_confidence > best_confidence:
                best_confidence = weighted_confidence
                best_digit = best_match_digit
                best_area = area
        
        # Eşik değeri düşürüldü - daha toleranslı
        if best_confidence > 0.1:  
            print(f"[DEBUG] Final result: digit {best_digit}, confidence: {best_confidence:.3f}")
            return best_digit, best_confidence
        else:
            print(f"[DEBUG] Confidence çok düşük: {best_confidence:.3f}")
            return None, 0.0
    
    def get_etap_from_digit(self, digit):
        """
        Rakamdan etap ismi döndür
        """
        etap_map = {
            1: "düz",
            2: "dik_eğim", 
            3: "hızlanma",
            4: "sığ_su"
        }
        return etap_map.get(digit, "düz")

    def save_debug_images(self, frame, digit=None):
        """Debug için görüntüleri kaydet"""
        try:
            # Orijinal
            cv2.imwrite(f'd:/barlas/debug_frame_original.jpg', frame)
            
            # Gray
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(f'd:/barlas/debug_frame_gray.jpg', gray)
            
            # CLAHE
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            gray_clahe = clahe.apply(gray)
            cv2.imwrite(f'd:/barlas/debug_frame_clahe.jpg', gray_clahe)
            
            # Thresholds
            blurred = cv2.GaussianBlur(gray_clahe, (5, 5), 0)
            
            thresh1 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY_INV, 11, 2)
            cv2.imwrite(f'd:/barlas/debug_frame_thresh_adaptive.jpg', thresh1)
            
            _, thresh2 = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            cv2.imwrite(f'd:/barlas/debug_frame_thresh_otsu.jpg', thresh2)
            
            print("[DEBUG] Debug görüntüleri d:/barlas/ klasörüne kaydedildi")
            
        except Exception as e:
            print(f"[DEBUG] Görüntü kaydetme hatası: {e}")

if __name__ == "__main__":
    # Test için
    recognizer = DigitRecognizer()
    
    print("Gelişmiş rakam tanıma sistemi hazır!")
    print(f"Her rakam için 3 template oluşturuldu: font kalın, font ince, 7-segment")
    
    # Template'leri göster
    for digit, templates in recognizer.digit_templates.items():
        for i, template in enumerate(templates):
            cv2.imshow(f'Digit {digit} - Template {i+1}', template)
    
    print("Template'leri görmek için herhangi bir tuşa basın...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
