#!/usr/bin/env python3
"""
BARLAS Traffic Sign Recognition System
Teknofest İnsansız Kara Aracı Yarışması - Tabela Algılama
Parkur görevlerini belirlemek için tabela tanıma
"""

import cv2
import numpy as np
import time
import os
import tensorflow as tf
from typing import List, Dict, Optional, Tuple

class TrafficSignRecognizer:
    """
    Teknofest yarışma tabelalarını tanıyan sistem
    - Dik eğim
    - Yan eğim  
    - Sığ su
    - Trafik konileri
    - Hızlanma
    - Dur
    """
    
    def __init__(self, model_path=None):
        """Traffic Sign Recognizer initialize"""
        self.sign_classes = {
            0: "dik_eğim",
            1: "yan_eğim", 
            2: "sığ_su",
            3: "trafik_konileri",
            4: "hızlanma",
            5: "dur",
            6: "çakıllı_yol",
            7: "engebeli_arazi"
        }
        
        # Model yükleme
        self.model = None
        if model_path and os.path.exists(model_path):
            try:
                self.model = tf.keras.models.load_model(model_path)
                print(f"✅ Tabela modeli yüklendi: {model_path}")
            except Exception as e:
                print(f"⚠️ Model yükleme hatası: {e}")
                self.model = None
        
        # Fallback: Renk/şekil bazlı tanıma
        if self.model is None:
            print("🔄 Renk/şekil bazlı tabela tanıma aktif")
            self.use_classical_detection = True
        else:
            self.use_classical_detection = False
        
        # Detection parametreleri
        self.confidence_threshold = 0.7
        self.detection_history = []
        self.stable_detection_frames = 5
        
        print("🚦 BARLAS Tabela Tanıma Sistemi hazır!")
    
    def detect_traffic_signs(self, frame) -> List[Dict]:
        """
        Framede trafik tabelalarını tespit et
        
        Args:
            frame: OpenCV BGR frame
            
        Returns:
            List[Dict]: Tespit edilen tabelalar
        """
        detections = []
        
        if self.use_classical_detection:
            detections = self._classical_sign_detection(frame)
        else:
            detections = self._deep_learning_detection(frame)
        
        # Stabil tespit için history kullan
        stable_detections = self._filter_stable_detections(detections)
        
        return stable_detections
    
    def _classical_sign_detection(self, frame) -> List[Dict]:
        """Renk ve şekil bazlı klasik tabela tanıma"""
        detections = []
        
        # Renk aralıkları (HSV)
        red_ranges = [
            ([0, 120, 70], [10, 255, 255]),    # Kırmızı alt aralık
            ([170, 120, 70], [180, 255, 255])  # Kırmızı üst aralık
        ]
        
        yellow_range = ([20, 100, 100], [30, 255, 255])  # Sarı
        blue_range = ([100, 150, 50], [130, 255, 255])   # Mavi
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Kırmızı tabelalar (dur, dik_eğim)
        red_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in red_ranges:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            red_mask = cv2.bitwise_or(red_mask, mask)
        
        red_signs = self._find_signs_in_mask(red_mask, frame, "red")
        detections.extend(red_signs)
        
        # Sarı tabelalar (hızlanma, dikkat)
        yellow_mask = cv2.inRange(hsv, np.array(yellow_range[0]), np.array(yellow_range[1]))
        yellow_signs = self._find_signs_in_mask(yellow_mask, frame, "yellow")
        detections.extend(yellow_signs)
        
        # Mavi tabelalar (bilgilendirme)
        blue_mask = cv2.inRange(hsv, np.array(blue_range[0]), np.array(blue_range[1]))
        blue_signs = self._find_signs_in_mask(blue_mask, frame, "blue")
        detections.extend(blue_signs)
        
        return detections
    
    def _find_signs_in_mask(self, mask, original_frame, color_type) -> List[Dict]:
        """Mask içinde tabela şekillerini bul"""
        signs = []
        
        # Morfolojik işlemler
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Konturları bul
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Minimum boyut kontrolü (çok küçük alanları filtrele)
            if area < 500:  # 500 piksel minimum alan
                continue
            
            # Bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Aspect ratio kontrolü (kare/dikdörtgen tabelalar)
            aspect_ratio = w / h
            if not (0.7 <= aspect_ratio <= 1.5):
                continue
            
            # Contour yaklaşımı (şekil analizi)
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # ROI çıkar
            roi = original_frame[y:y+h, x:x+w]
            
            # Tabela sınıfını belirle
            sign_class = self._classify_sign_by_shape_color(roi, color_type, len(approx))
            
            if sign_class:
                sign_info = {
                    'bbox': [x, y, w, h],
                    'confidence': 0.8,  # Classical detection için sabit güven
                    'class': sign_class,
                    'center': (x + w//2, y + h//2),
                    'area': area,
                    'timestamp': time.time()
                }
                signs.append(sign_info)
        
        return signs
    
    def _classify_sign_by_shape_color(self, roi, color_type, vertex_count) -> Optional[str]:
        """ROI'deki tabela sınıfını şekil ve renge göre belirle"""
        
        # Kırmızı tabelalar
        if color_type == "red":
            if vertex_count >= 8:  # Dairesel (dur işareti)
                return "dur"
            elif 3 <= vertex_count <= 6:  # Üçgen/çokgen (uyarı)
                return "dik_eğim"
        
        # Sarı tabelalar  
        elif color_type == "yellow":
            if vertex_count >= 6:  # Çokgen (dikkat)
                # ROI'da ok analizi yapılabilir
                return "hızlanma"
            else:
                return "trafik_konileri"
        
        # Mavi tabelalar
        elif color_type == "blue":
            return "sığ_su"  # Mavi genelde bilgilendirme
        
        return None
    
    def _deep_learning_detection(self, frame) -> List[Dict]:
        """Derin öğrenme tabanlı tabela tanıma"""
        detections = []
        
        try:
            # Frame'i model için hazırla
            input_size = (224, 224)  # Model input boyutu
            resized = cv2.resize(frame, input_size)
            normalized = resized / 255.0
            input_batch = np.expand_dims(normalized, axis=0)
            
            # Tahmin yap
            predictions = self.model.predict(input_batch, verbose=0)
            
            # En yüksek güvenli sınıfı al
            max_confidence_idx = np.argmax(predictions[0])
            max_confidence = predictions[0][max_confidence_idx]
            
            if max_confidence >= self.confidence_threshold:
                sign_class = self.sign_classes.get(max_confidence_idx, "unknown")
                
                # Basit bir bounding box (tam frame)
                h, w = frame.shape[:2]
                
                detection = {
                    'bbox': [w//4, h//4, w//2, h//2],  # Merkez bölge
                    'confidence': float(max_confidence),
                    'class': sign_class,
                    'center': (w//2, h//2),
                    'area': (w//2) * (h//2),
                    'timestamp': time.time()
                }
                detections.append(detection)
        
        except Exception as e:
            print(f"⚠️ Deep learning detection hatası: {e}")
        
        return detections
    
    def _filter_stable_detections(self, current_detections) -> List[Dict]:
        """Kararlı tespitler için history filtresi"""
        
        # Mevcut tespitleri history'ye ekle
        self.detection_history.append({
            'timestamp': time.time(),
            'detections': current_detections
        })
        
        # Eski kayıtları temizle (5 saniyeden eski)
        current_time = time.time()
        self.detection_history = [
            entry for entry in self.detection_history 
            if current_time - entry['timestamp'] <= 5.0
        ]
        
        # Kararlı tespitler için oylama sistemi
        stable_signs = []
        
        for detection in current_detections:
            # Aynı bölgede kaç kez görüldüğünü say
            similar_count = 0
            
            for history_entry in self.detection_history[-self.stable_detection_frames:]:
                for hist_det in history_entry['detections']:
                    if (hist_det['class'] == detection['class'] and 
                        self._is_similar_location(detection, hist_det)):
                        similar_count += 1
            
            # Kararlı tespit kriteria (en az 3/5 frame)
            if similar_count >= 3:
                stable_signs.append(detection)
        
        return stable_signs
    
    def _is_similar_location(self, det1, det2, threshold=50) -> bool:
        """İki tespitin aynı bölgede olup olmadığını kontrol et"""
        center1 = det1['center']
        center2 = det2['center']
        
        distance = ((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)**0.5
        return distance <= threshold
    
    def get_mission_command_from_sign(self, sign_class) -> Dict:
        """Tabela sınıfından mission komutu üret"""
        
        mission_commands = {
            "dik_eğim": {
                "action": "slow_climb",
                "speed_limit": 0.3,
                "brake_mode": "active",
                "pid_gains": {"kp": 1.0, "ki": 0.12, "kd": 0.06}
            },
            "yan_eğim": {
                "action": "side_slope",
                "speed_limit": 0.4,
                "brake_mode": "ready",
                "pid_gains": {"kp": 0.8, "ki": 0.10, "kd": 0.05}
            },
            "sığ_su": {
                "action": "water_crossing",
                "speed_limit": 0.25,
                "brake_mode": "disabled",
                "pid_gains": {"kp": 0.9, "ki": 0.08, "kd": 0.04}
            },
            "trafik_konileri": {
                "action": "slalom_mode",
                "speed_limit": 0.35,
                "brake_mode": "ready", 
                "pid_gains": {"kp": 0.9, "ki": 0.04, "kd": 0.05}
            },
            "hızlanma": {
                "action": "acceleration_zone",
                "speed_limit": 1.0,
                "brake_mode": "disabled",
                "pid_gains": {"kp": 0.6, "ki": 0.08, "kd": 0.03}
            },
            "dur": {
                "action": "stop_and_wait",
                "speed_limit": 0.0,
                "brake_mode": "active",
                "wait_duration": 3.0
            },
            "çakıllı_yol": {
                "action": "rough_terrain",
                "speed_limit": 0.4,
                "brake_mode": "ready",
                "pid_gains": {"kp": 1.0, "ki": 0.12, "kd": 0.06}
            }
        }
        
        return mission_commands.get(sign_class, {
            "action": "default_drive",
            "speed_limit": 0.5,
            "brake_mode": "ready",
            "pid_gains": {"kp": 0.6, "ki": 0.08, "kd": 0.03}
        })
    
    def draw_detections(self, frame, detections) -> np.ndarray:
        """Tespitleri frame üzerine çiz"""
        display_frame = frame.copy()
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            sign_class = detection['class']
            
            # Bounding box
            color = (0, 255, 0)  # Yeşil
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), color, 2)
            
            # Label
            label = f"{sign_class}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            
            # Label background
            cv2.rectangle(display_frame, (x, y-label_size[1]-10), 
                         (x+label_size[0], y), color, -1)
            
            # Label text
            cv2.putText(display_frame, label, (x, y-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Merkez nokta
            center = detection['center']
            cv2.circle(display_frame, center, 5, (255, 0, 0), -1)
        
        return display_frame

def test_traffic_sign_recognizer():
    """Test fonksiyonu"""
    print("🚦 BARLAS Traffic Sign Recognition Test")
    print("=" * 50)
    
    recognizer = TrafficSignRecognizer()
    
    # Kamera başlat
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Kamera açılamadı!")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("🚀 Tabela tanıma testi başlıyor...")
    print("📋 Kontroller:")
    print("  'q' - Çıkış")
    print("  's' - Screenshot al")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Tabela tespiti
        detections = recognizer.detect_traffic_signs(frame)
        
        # Çizim
        display_frame = recognizer.draw_detections(frame, detections)
        
        # Tespit bilgileri
        if detections:
            for i, detection in enumerate(detections):
                mission_cmd = recognizer.get_mission_command_from_sign(detection['class'])
                print(f"Tabela {i+1}: {detection['class']} -> {mission_cmd['action']}")
        
        cv2.imshow('BARLAS Traffic Sign Recognition', display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"traffic_sign_test_{int(time.time())}.jpg"
            cv2.imwrite(filename, display_frame)
            print(f"Screenshot kaydedildi: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    print("✅ Test tamamlandı!")

if __name__ == "__main__":
    test_traffic_sign_recognizer()
