"""
Eğitilmiş CNN Model ile Tabela Tanıma
PyTorch veya TensorFlow modelini kullanır
"""
import cv2
import numpy as np
import os
import glob
import json

# Framework imports
try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    PYTORCH_AVAILABLE = True
except ImportError:
    PYTORCH_AVAILABLE = False

try:
    import tensorflow as tf
    TENSORFLOW_AVAILABLE = True
except ImportError:
    TENSORFLOW_AVAILABLE = False

class DigitCNN(nn.Module):
    """PyTorch CNN modeli (eğitim ile aynı architecture)"""
    def __init__(self, num_classes=4):
        super(DigitCNN, self).__init__()
        
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.dropout = nn.Dropout(0.5)
        self.fc1 = nn.Linear(128 * 8 * 8, 512)
        self.fc2 = nn.Linear(512, 128)
        self.fc3 = nn.Linear(128, num_classes)
    
    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = self.pool(F.relu(self.conv3(x)))
        x = x.view(-1, 128 * 8 * 8)
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.dropout(x)
        x = self.fc3(x)
        return x

class TrainedDigitRecognizer:
    """Eğitilmiş model ile rakam tanıma"""
    
    def __init__(self, models_path="d:/barlas/dataset/models"):
        """
        Eğitilmiş modeli yükle
        """
        self.models_path = models_path
        self.model = None
        self.framework = None
        self.class_names = ['digit_1', 'digit_2', 'digit_3', 'digit_4']
        self.device = None
        
        print("🤖 Eğitilmiş model yükleniyor...")
        self._load_best_model()
    
    def _load_best_model(self):
        """En iyi modeli bul ve yükle"""
        if not os.path.exists(self.models_path):
            print(f"❌ Model klasörü bulunamadı: {self.models_path}")
            return False
        
        # PyTorch modellerini bul
        pytorch_models = glob.glob(os.path.join(self.models_path, "pytorch_*.pth"))
        tensorflow_models = glob.glob(os.path.join(self.models_path, "tensorflow_*.h5"))
        
        best_model_path = None
        best_accuracy = 0
        best_framework = None
        
        # PyTorch modellerini kontrol et
        if PYTORCH_AVAILABLE and pytorch_models:
            print("   📦 PyTorch modelleri kontrol ediliyor...")
            
            for model_path in pytorch_models:
                try:
                    checkpoint = torch.load(model_path, map_location='cpu')
                    accuracy = checkpoint.get('test_accuracy', 0)
                    
                    print(f"      {os.path.basename(model_path)}: {accuracy:.2f}%")
                    
                    if accuracy > best_accuracy:
                        best_accuracy = accuracy
                        best_model_path = model_path
                        best_framework = 'pytorch'
                        
                except Exception as e:
                    print(f"      ❌ Hata: {model_path} - {e}")
        
        # TensorFlow modellerini kontrol et
        if TENSORFLOW_AVAILABLE and tensorflow_models:
            print("   📦 TensorFlow modelleri kontrol ediliyor...")
            
            # TensorFlow modellerinin accuracy'sini results.json'dan al
            results_file = os.path.join(self.models_path, 'training_results.json')
            tf_accuracy = 0
            
            if os.path.exists(results_file):
                try:
                    with open(results_file, 'r') as f:
                        results = json.load(f)
                    tf_accuracy = results.get('tensorflow_accuracy', 0)
                except:
                    tf_accuracy = 0
            
            if tf_accuracy > best_accuracy:
                # En son TensorFlow modelini seç
                latest_tf_model = max(tensorflow_models, key=os.path.getctime)
                best_accuracy = tf_accuracy
                best_model_path = latest_tf_model
                best_framework = 'tensorflow'
                print(f"      {os.path.basename(latest_tf_model)}: {tf_accuracy:.2f}%")
        
        # En iyi modeli yükle
        if best_model_path:
            print(f"✅ En iyi model: {os.path.basename(best_model_path)} ({best_accuracy:.2f}%)")
            
            if best_framework == 'pytorch':
                return self._load_pytorch_model(best_model_path)
            elif best_framework == 'tensorflow':
                return self._load_tensorflow_model(best_model_path)
        else:
            print("❌ Hiçbir uygun model bulunamadı!")
            return False
    
    def _load_pytorch_model(self, model_path):
        """PyTorch modeli yükle"""
        try:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            
            # Model oluştur
            self.model = DigitCNN(num_classes=4)
            
            # Checkpoint yükle
            checkpoint = torch.load(model_path, map_location=self.device)
            self.model.load_state_dict(checkpoint['model_state_dict'])
            self.model.to(self.device)
            self.model.eval()
            
            self.framework = 'pytorch'
            print(f"   ✅ PyTorch modeli yüklendi ({self.device})")
            return True
            
        except Exception as e:
            print(f"   ❌ PyTorch model yükleme hatası: {e}")
            return False
    
    def _load_tensorflow_model(self, model_path):
        """TensorFlow modeli yükle"""
        try:
            self.model = tf.keras.models.load_model(model_path)
            self.framework = 'tensorflow'
            print(f"   ✅ TensorFlow modeli yüklendi")
            return True
            
        except Exception as e:
            print(f"   ❌ TensorFlow model yükleme hatası: {e}")
            return False
    
    def _preprocess_image(self, img):
        """Görüntü ön işleme (eğitim ile aynı)"""
        # ROI detection ile rakam bölgesini bul
        digit_regions = self._detect_digit_regions(img)
        
        if not digit_regions:
            # Eğer ROI bulunamazsa tüm görüntüyü kullan
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            processed = cv2.resize(gray, (64, 64))
            return processed.astype(np.float32) / 255.0
        
        # En büyük ROI'yi seç
        best_roi = max(digit_regions, key=lambda x: x[2] * x[3])
        x, y, w, h = best_roi
        
        # ROI'yi çıkar
        roi = img[y:y+h, x:x+w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Boyut standardizasyonu
        resized = cv2.resize(gray, (64, 64))
        
        # Normalizasyon
        normalized = resized.astype(np.float32) / 255.0
        
        return normalized
    
    def _detect_digit_regions(self, img):
        """Görüntüdeki rakam bölgelerini tespit et"""
        # Gri tonlama
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # CLAHE uygula
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        # Threshold
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # Morphological operations
        kernel = np.ones((3,3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Contour bul
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        digit_regions = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Alan filtresi
            if area < 100:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            # Aspect ratio filtresi
            aspect_ratio = h / w
            if aspect_ratio < 0.8 or aspect_ratio > 5.0:
                continue
            
            # Boyut filtresi
            if w < 10 or h < 15:
                continue
            
            digit_regions.append((x, y, w, h))
        
        return digit_regions
    
    def recognize_digit_in_frame(self, frame):
        """Frame'den rakam tanı"""
        if self.model is None:
            print("❌ Model yüklenmemiş!")
            return None, 0.0
        
        try:
            # Görüntüyü işle
            processed_img = self._preprocess_image(frame)
            
            if self.framework == 'pytorch':
                return self._predict_pytorch(processed_img)
            elif self.framework == 'tensorflow':
                return self._predict_tensorflow(processed_img)
            else:
                return None, 0.0
                
        except Exception as e:
            print(f"❌ Tanıma hatası: {e}")
            return None, 0.0
    
    def _predict_pytorch(self, img):
        """PyTorch ile tahmin"""
        # Tensor'a çevir
        img_tensor = torch.FloatTensor(img).unsqueeze(0).unsqueeze(0).to(self.device)
        
        # Tahmin
        with torch.no_grad():
            outputs = self.model(img_tensor)
            probabilities = F.softmax(outputs, dim=1)
            confidence, predicted = torch.max(probabilities, 1)
            
            digit = predicted.item() + 1  # 0-3 -> 1-4
            confidence = confidence.item()
            
            return digit, confidence
    
    def _predict_tensorflow(self, img):
        """TensorFlow ile tahmin"""
        # Reshape
        img_input = img.reshape(1, 64, 64, 1)
        
        # Tahmin
        predictions = self.model.predict(img_input, verbose=0)
        
        predicted_class = np.argmax(predictions[0])
        confidence = np.max(predictions[0])
        
        digit = predicted_class + 1  # 0-3 -> 1-4
        
        return digit, confidence
    
    def get_etap_from_digit(self, digit):
        """Rakamdan etap ismi döndür"""
        etap_map = {
            1: "düz",
            2: "dik_eğim", 
            3: "hızlanma",
            4: "sığ_su"
        }
        return etap_map.get(digit, "düz")
    
    def is_model_loaded(self):
        """Model yüklü mü kontrol et"""
        return self.model is not None

# Test fonksiyonu
def test_trained_model():
    """Eğitilmiş modeli test et"""
    recognizer = TrainedDigitRecognizer()
    
    if not recognizer.is_model_loaded():
        print("❌ Model yüklenemedi! Önce model eğitimi yapın.")
        return
    
    print("🎯 Model test ediliyor...")
    
    # Test görüntüsü oluştur
    test_img = np.ones((200, 200, 3), dtype=np.uint8) * 255
    cv2.putText(test_img, '3', (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 0), 8)
    
    # Tanıma
    digit, confidence = recognizer.recognize_digit_in_frame(test_img)
    etap = recognizer.get_etap_from_digit(digit)
    
    print(f"🎯 Sonuç: Rakam {digit}, Etap: {etap}, Güven: {confidence:.3f}")

if __name__ == "__main__":
    test_trained_model()
