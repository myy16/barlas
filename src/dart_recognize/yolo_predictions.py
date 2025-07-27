#!/usr/bin/env python
# coding: utf-8

"""
BARLAS YOLO Dart Detection System
Jupyter Notebook'tan tam dönüştürülmüş modüler versiyon
"""
import cv2
import numpy as np
import os 
import yaml
from yaml.loader import SafeLoader
from typing import List, Dict, Tuple, Optional

class YOLOPredictions:
    def __init__(self, onnx_model_path=None, data_yaml_path=None):   
        
        # Default model ve config paths
        if onnx_model_path is None:
            onnx_model_path = r"D:\yolo\yolo_object_detection\Model\weights\best.onnx"
        if data_yaml_path is None:
            data_yaml_path = 'data.yaml'
        
        # YAML config dosyasını yükle
        try:
            with open(data_yaml_path, mode='r') as f:
                data_yaml = yaml.load(f, Loader=SafeLoader)
            
            self.labels = data_yaml['names']
            self.nc = data_yaml['nc']
        except FileNotFoundError:
            # Default labels if data.yaml not found
            self.labels = {0: 'dart'}
            self.nc = 1
            print(f"Warning: {data_yaml_path} not found, using default labels")
        
        # YOLO modelini yükle
        if not os.path.exists(onnx_model_path):
            raise FileNotFoundError(f"YOLO model file not found: {onnx_model_path}")
            
        self.yolo = cv2.dnn.readNetFromONNX(onnx_model_path)
        self.yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        self.INPUT_WH_YOLO = 640
        self.confidence_threshold = 0.4
        self.class_threshold = 0.25
        self.nms_threshold = 0.45
    
    def predictions(self, image, draw_boxes=True):
        """
        Görüntüden dart tespiti yapar ve sonuçları döndürür
        
        Args:
            image: OpenCV image (numpy array)
            draw_boxes: True ise bounding box'ları çizer
            
        Returns:
            Processed image with bounding boxes (if draw_boxes=True)
        """
        
        row, col, d = image.shape
        
        # Görüntüyü kare hale getir
        max_rc = max(row, col)
        input_image = np.zeros((max_rc, max_rc, 3), dtype=np.uint8)
        input_image[0:row, 0:col] = image
        
        # YOLO tahmin
        blob = cv2.dnn.blobFromImage(input_image, 1/255, (self.INPUT_WH_YOLO, self.INPUT_WH_YOLO), 
                                   swapRB=True, crop=False)
        self.yolo.setInput(blob)
        preds = self.yolo.forward()
        
        # NMS (Non Maximum Suppression)
        detections = preds[0]
        boxes = []
        confidences = []
        classes = []
        
        # Ölçekleme faktörleri
        image_w, image_h = input_image.shape[:2]
        x_factor = image_w / self.INPUT_WH_YOLO
        y_factor = image_h / self.INPUT_WH_YOLO
        
        for i in range(len(detections)):
            row = detections[i]
            confidence = row[4]
            
            if confidence > self.confidence_threshold:
                class_score = row[5:].max()
                class_id = row[5:].argmax()
                
                if class_score > self.class_threshold:
                    cx, cy, w, h = row[0:4]
                    
                    # Bounding box koordinatları
                    left = int((cx - 0.5*w) * x_factor)
                    top = int((cy - 0.5*h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    
                    box = np.array([left, top, width, height])
                    
                    confidences.append(confidence)
                    boxes.append(box)
                    classes.append(class_id)
        
        # NMS uygula
        boxes_np = np.array(boxes).tolist()
        confidences_np = np.array(confidences).tolist()
        
        if len(boxes_np) > 0:
            index = np.array(cv2.dnn.NMSBoxes(boxes_np, confidences_np, 
                                            self.class_threshold, self.nms_threshold)).flatten()
            
            # Bounding box'ları çiz
            if draw_boxes and len(index) > 0:
                for ind in index:
                    x, y, w, h = boxes_np[ind]
                    bb_conf = int(confidences_np[ind] * 100)
                    classes_id = classes[ind]
                    class_name = self.labels[classes_id]
                    colors = self.generate_colors(classes_id)
                    
                    text = f'{class_name}: {bb_conf}%'
                    
                    cv2.rectangle(image, (x, y), (x+w, y+h), colors, 2)
                    cv2.rectangle(image, (x, y-30), (x+w, y), colors, -1)
                    cv2.putText(image, text, (x, y-10), cv2.FONT_HERSHEY_PLAIN, 0.7, (0, 0, 0), 1)
            
            return image
        
        return image

    def get_detections(self, image):
        """
        Sadece tespit verilerini döndürür (bounding box çizmez)
        
        Args:
            image: OpenCV image (numpy array)
            
        Returns:
            List of detection dictionaries: 
            [{'bbox': [x, y, w, h], 'confidence': float, 'class_id': int, 'class_name': str}]
        """
        
        row, col, d = image.shape
        
        # Görüntüyü kare hale getir
        max_rc = max(row, col)
        input_image = np.zeros((max_rc, max_rc, 3), dtype=np.uint8)
        input_image[0:row, 0:col] = image
        
        # YOLO tahmin
        blob = cv2.dnn.blobFromImage(input_image, 1/255, (self.INPUT_WH_YOLO, self.INPUT_WH_YOLO), 
                                   swapRB=True, crop=False)
        self.yolo.setInput(blob)
        preds = self.yolo.forward()
        
        # NMS (Non Maximum Suppression)
        detections = preds[0]
        boxes = []
        confidences = []
        classes = []
        
        # Ölçekleme faktörleri
        image_w, image_h = input_image.shape[:2]
        x_factor = image_w / self.INPUT_WH_YOLO
        y_factor = image_h / self.INPUT_WH_YOLO
        
        for i in range(len(detections)):
            row = detections[i]
            confidence = row[4]
            
            if confidence > self.confidence_threshold:
                class_score = row[5:].max()
                class_id = row[5:].argmax()
                
                if class_score > self.class_threshold:
                    cx, cy, w, h = row[0:4]
                    
                    # Bounding box koordinatları
                    left = int((cx - 0.5*w) * x_factor)
                    top = int((cy - 0.5*h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    
                    box = np.array([left, top, width, height])
                    
                    confidences.append(confidence)
                    boxes.append(box)
                    classes.append(class_id)
        
        # NMS uygula
        boxes_np = np.array(boxes).tolist()
        confidences_np = np.array(confidences).tolist()
        
        detection_results = []
        
        if len(boxes_np) > 0:
            index = np.array(cv2.dnn.NMSBoxes(boxes_np, confidences_np, 
                                            self.class_threshold, self.nms_threshold)).flatten()
            
            for ind in index:
                x, y, w, h = boxes_np[ind]
                confidence = confidences_np[ind]
                class_id = classes[ind]
                class_name = self.labels[class_id]
                
                detection_results.append({
                    'bbox': [x, y, w, h],
                    'confidence': confidence,
                    'class_id': class_id,
                    'class_name': class_name
                })
        
        return detection_results
    
    def get_largest_detection(self, image):
        """
        En büyük tespit edilen nesneyi döndürür (dart hedefleme için)
        
        Args:
            image: OpenCV image (numpy array)
            
        Returns:
            Detection dictionary veya None: 
            {'bbox': [x, y, w, h], 'confidence': float, 'class_id': int, 'class_name': str, 'center': (cx, cy)}
        """
        detections = self.get_detections(image)
        
        if not detections:
            return None
        
        # En büyük area'ya sahip detection'ı bul
        largest_detection = max(detections, key=lambda d: d['bbox'][2] * d['bbox'][3])
        
        # Merkez koordinatları ekle
        x, y, w, h = largest_detection['bbox']
        center_x = x + w // 2
        center_y = y + h // 2
        largest_detection['center'] = (center_x, center_y)
        
        return largest_detection

    def generate_colors(self, ID):
        """Sınıf ID'si için rastgele renk üretir"""
        np.random.seed(10)
        colors = np.random.randint(100, 255, size=(self.nc, 3)).tolist()
        return tuple(colors[ID])
    
    def set_thresholds(self, confidence_threshold=0.4, class_threshold=0.25, nms_threshold=0.45):
        """Tespit eşiklerini ayarlar"""
        self.confidence_threshold = confidence_threshold
        self.class_threshold = class_threshold
        self.nms_threshold = nms_threshold

# Test için ana fonksiyon
if __name__ == "__main__":
    # YOLO nesnesini oluştur
    yolo = YOLOPredictions()
    
    # Test kamerası
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Kamera açılamadı!")
        exit()
    
    print("Dart tespit sistemi başlatılıyor...")
    print("Çıkmak için 'q' tuşuna basın")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Tahmin yap ve çiz
        result_image = yolo.predictions(frame)
        
        # En büyük dart tespitini göster
        largest_dart = yolo.get_largest_detection(frame)
        if largest_dart:
            center_x, center_y = largest_dart['center']
            confidence = largest_dart['confidence']
            cv2.circle(result_image, (center_x, center_y), 10, (0, 255, 0), -1)
            cv2.putText(result_image, f"Target: {confidence:.2f}", 
                       (center_x-50, center_y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow('BARLAS Dart Detection', result_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()