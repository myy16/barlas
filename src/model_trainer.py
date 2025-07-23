"""
BARLAS Tabela CNN Model Eğitimi
Toplanan veri setini kullanarak CNN modeli eğitir
"""
import os
import cv2
import numpy as np
import json
from datetime import datetime
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
import seaborn as sns

# PyTorch import (eğer yoksa TensorFlow kullanacağız)
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    import torch.nn.functional as F
    from torch.utils.data import Dataset, DataLoader
    import torchvision.transforms as transforms
    PYTORCH_AVAILABLE = True
    print("✅ PyTorch kullanılacak")
except ImportError:
    PYTORCH_AVAILABLE = False
    print("⚠️ PyTorch yok, TensorFlow deneniyor...")
    
    try:
        import tensorflow as tf
        from tensorflow import keras
        from tensorflow.keras import layers
        TENSORFLOW_AVAILABLE = True
        print("✅ TensorFlow kullanılacak")
    except ImportError:
        TENSORFLOW_AVAILABLE = False
        print("❌ Ne PyTorch ne de TensorFlow bulunamadı!")

class DigitDataset:
    """Veri seti yükleme ve işleme sınıfı"""
    
    def __init__(self, dataset_path):
        """
        Dataset'i yükle
        """
        self.dataset_path = dataset_path
        self.labeled_path = os.path.join(dataset_path, "labeled_data")
        self.images = []
        self.labels = []
        self.class_names = ['digit_1', 'digit_2', 'digit_3', 'digit_4']
        
        self._load_dataset()
    
    def _load_dataset(self):
        """Etiketli veri setini yükle"""
        print("📂 Veri seti yükleniyor...")
        
        for class_idx, class_name in enumerate(self.class_names):
            class_path = os.path.join(self.labeled_path, class_name)
            
            if not os.path.exists(class_path):
                print(f"⚠️ Klasör bulunamadı: {class_path}")
                continue
            
            image_files = [f for f in os.listdir(class_path) if f.endswith('.jpg')]
            print(f"   {class_name}: {len(image_files)} görüntü")
            
            for img_file in image_files:
                img_path = os.path.join(class_path, img_file)
                
                # Görüntüyü yükle ve işle
                img = cv2.imread(img_path)
                if img is not None:
                    # Preprocessing
                    img_processed = self._preprocess_image(img)
                    self.images.append(img_processed)
                    self.labels.append(class_idx)
        
        self.images = np.array(self.images)
        self.labels = np.array(self.labels)
        
        print(f"✅ Toplam {len(self.images)} görüntü yüklendi")
        print(f"   Sınıf dağılımı: {np.bincount(self.labels)}")
    
    def _preprocess_image(self, img):
        """Görüntü ön işleme"""
        # Gri tonlama
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Boyut standardizasyonu
        resized = cv2.resize(gray, (64, 64))
        
        # Normalizasyon
        normalized = resized.astype(np.float32) / 255.0
        
        return normalized
    
    def get_train_test_split(self, test_size=0.2, random_state=42):
        """Train/test split"""
        return train_test_split(
            self.images, self.labels, 
            test_size=test_size, 
            random_state=random_state,
            stratify=self.labels
        )

# PyTorch CNN Model
if PYTORCH_AVAILABLE:
    class DigitCNN(nn.Module):
        def __init__(self, num_classes=4):
            super(DigitCNN, self).__init__()
            
            # Convolutional layers
            self.conv1 = nn.Conv2d(1, 32, kernel_size=3, padding=1)
            self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
            self.conv3 = nn.Conv2d(64, 128, kernel_size=3, padding=1)
            
            # Pooling
            self.pool = nn.MaxPool2d(2, 2)
            
            # Dropout
            self.dropout = nn.Dropout(0.5)
            
            # Fully connected layers
            self.fc1 = nn.Linear(128 * 8 * 8, 512)
            self.fc2 = nn.Linear(512, 128)
            self.fc3 = nn.Linear(128, num_classes)
        
        def forward(self, x):
            # Conv layers
            x = self.pool(F.relu(self.conv1(x)))
            x = self.pool(F.relu(self.conv2(x)))
            x = self.pool(F.relu(self.conv3(x)))
            
            # Flatten
            x = x.view(-1, 128 * 8 * 8)
            
            # FC layers
            x = F.relu(self.fc1(x))
            x = self.dropout(x)
            x = F.relu(self.fc2(x))
            x = self.dropout(x)
            x = self.fc3(x)
            
            return x

class ModelTrainer:
    """Model eğitim sınıfı"""
    
    def __init__(self, dataset_path):
        """
        Trainer'ı başlat
        """
        self.dataset_path = dataset_path
        self.models_path = os.path.join(dataset_path, "models")
        os.makedirs(self.models_path, exist_ok=True)
        
        # Veri setini yükle
        self.dataset = DigitDataset(dataset_path)
        
        if len(self.dataset.images) == 0:
            raise ValueError("Veri seti boş! Önce veri toplama yapın.")
        
        # Train/test split
        self.X_train, self.X_test, self.y_train, self.y_test = self.dataset.get_train_test_split()
        
        print(f"📊 Eğitim seti: {len(self.X_train)} örnek")
        print(f"📊 Test seti: {len(self.X_test)} örnek")
    
    def train_pytorch_model(self, epochs=50, batch_size=32, learning_rate=0.001):
        """PyTorch modeli eğit"""
        if not PYTORCH_AVAILABLE:
            print("❌ PyTorch mevcut değil!")
            return None
        
        print("🔥 PyTorch modeli eğitiliyor...")
        
        # Device
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"   Device: {device}")
        
        # Model
        model = DigitCNN(num_classes=4).to(device)
        
        # Loss ve optimizer
        criterion = nn.CrossEntropyLoss()
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        
        # Veriyi tensor'a çevir
        X_train_tensor = torch.FloatTensor(self.X_train).unsqueeze(1).to(device)
        y_train_tensor = torch.LongTensor(self.y_train).to(device)
        X_test_tensor = torch.FloatTensor(self.X_test).unsqueeze(1).to(device)
        y_test_tensor = torch.LongTensor(self.y_test).to(device)
        
        # Eğitim döngüsü
        train_losses = []
        train_accuracies = []
        
        for epoch in range(epochs):
            model.train()
            
            # Mini-batch eğitimi
            epoch_loss = 0
            correct = 0
            total = 0
            
            # Basit batch işleme
            for i in range(0, len(X_train_tensor), batch_size):
                batch_X = X_train_tensor[i:i+batch_size]
                batch_y = y_train_tensor[i:i+batch_size]
                
                optimizer.zero_grad()
                outputs = model(batch_X)
                loss = criterion(outputs, batch_y)
                loss.backward()
                optimizer.step()
                
                epoch_loss += loss.item()
                _, predicted = torch.max(outputs.data, 1)
                total += batch_y.size(0)
                correct += (predicted == batch_y).sum().item()
            
            # Epoch sonucu
            avg_loss = epoch_loss / (len(X_train_tensor) // batch_size)
            accuracy = 100 * correct / total
            
            train_losses.append(avg_loss)
            train_accuracies.append(accuracy)
            
            if (epoch + 1) % 10 == 0:
                print(f"   Epoch {epoch+1}/{epochs} - Loss: {avg_loss:.4f}, Accuracy: {accuracy:.2f}%")
        
        # Test değerlendirmesi
        model.eval()
        with torch.no_grad():
            test_outputs = model(X_test_tensor)
            _, test_predicted = torch.max(test_outputs.data, 1)
            test_accuracy = 100 * (test_predicted == y_test_tensor).sum().item() / len(y_test_tensor)
        
        print(f"🎯 Test Accuracy: {test_accuracy:.2f}%")
        
        # Model kaydet
        model_path = os.path.join(self.models_path, f"pytorch_digit_model_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pth")
        torch.save({
            'model_state_dict': model.state_dict(),
            'test_accuracy': test_accuracy,
            'class_names': self.dataset.class_names,
            'train_losses': train_losses,
            'train_accuracies': train_accuracies
        }, model_path)
        
        print(f"💾 Model kaydedildi: {model_path}")
        
        # Eğitim grafiklerini çiz
        self._plot_training_history(train_losses, train_accuracies, "PyTorch")
        
        return model, test_accuracy
    
    def train_tensorflow_model(self, epochs=50, batch_size=32):
        """TensorFlow modeli eğit"""
        if not TENSORFLOW_AVAILABLE:
            print("❌ TensorFlow mevcut değil!")
            return None
        
        print("🔥 TensorFlow modeli eğitiliyor...")
        
        # Veriyi reshape et
        X_train_tf = self.X_train.reshape(-1, 64, 64, 1)
        X_test_tf = self.X_test.reshape(-1, 64, 64, 1)
        
        # One-hot encoding
        y_train_tf = tf.keras.utils.to_categorical(self.y_train, 4)
        y_test_tf = tf.keras.utils.to_categorical(self.y_test, 4)
        
        # Model oluştur
        model = tf.keras.Sequential([
            layers.Conv2D(32, (3, 3), activation='relu', input_shape=(64, 64, 1)),
            layers.MaxPooling2D((2, 2)),
            layers.Conv2D(64, (3, 3), activation='relu'),
            layers.MaxPooling2D((2, 2)),
            layers.Conv2D(128, (3, 3), activation='relu'),
            layers.MaxPooling2D((2, 2)),
            layers.Flatten(),
            layers.Dense(512, activation='relu'),
            layers.Dropout(0.5),
            layers.Dense(128, activation='relu'),
            layers.Dropout(0.5),
            layers.Dense(4, activation='softmax')
        ])
        
        # Compile
        model.compile(
            optimizer='adam',
            loss='categorical_crossentropy',
            metrics=['accuracy']
        )
        
        # Eğit
        history = model.fit(
            X_train_tf, y_train_tf,
            batch_size=batch_size,
            epochs=epochs,
            validation_data=(X_test_tf, y_test_tf),
            verbose=1
        )
        
        # Test accuracy
        test_loss, test_accuracy = model.evaluate(X_test_tf, y_test_tf, verbose=0)
        test_accuracy *= 100
        
        print(f"🎯 Test Accuracy: {test_accuracy:.2f}%")
        
        # Model kaydet
        model_path = os.path.join(self.models_path, f"tensorflow_digit_model_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h5")
        model.save(model_path)
        
        print(f"💾 Model kaydedildi: {model_path}")
        
        # Eğitim grafiklerini çiz
        self._plot_training_history(history.history['loss'], history.history['accuracy'], "TensorFlow")
        
        return model, test_accuracy
    
    def _plot_training_history(self, losses, accuracies, framework):
        """Eğitim grafiklerini çiz"""
        try:
            plt.figure(figsize=(12, 4))
            
            # Loss plot
            plt.subplot(1, 2, 1)
            plt.plot(losses)
            plt.title(f'{framework} - Training Loss')
            plt.xlabel('Epoch')
            plt.ylabel('Loss')
            
            # Accuracy plot
            plt.subplot(1, 2, 2)
            plt.plot(accuracies)
            plt.title(f'{framework} - Training Accuracy')
            plt.xlabel('Epoch')
            plt.ylabel('Accuracy (%)')
            
            plt.tight_layout()
            
            # Kaydet
            plot_path = os.path.join(self.models_path, f"training_history_{framework.lower()}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            plt.savefig(plot_path)
            plt.show()
            
            print(f"📊 Eğitim grafiği kaydedildi: {plot_path}")
            
        except Exception as e:
            print(f"⚠️ Grafik çizme hatası: {e}")

def main():
    """Ana fonksiyon"""
    dataset_path = "d:/barlas/dataset"
    
    print("🤖 BARLAS Tabela CNN Model Eğitimi")
    print("=" * 50)
    
    try:
        # Trainer oluştur
        trainer = ModelTrainer(dataset_path)
        
        # Model seçimi
        if PYTORCH_AVAILABLE:
            print("\n1. PyTorch modeli eğitiliyor...")
            pytorch_model, pytorch_acc = trainer.train_pytorch_model(epochs=30)
        
        if TENSORFLOW_AVAILABLE:
            print("\n2. TensorFlow modeli eğitiliyor...")
            tf_model, tf_acc = trainer.train_tensorflow_model(epochs=30)
        
        print("\n🎉 Model eğitimi tamamlandı!")
        
        # Sonuçları kaydet
        results = {
            'training_date': datetime.now().isoformat(),
            'dataset_size': len(trainer.dataset.images),
            'train_size': len(trainer.X_train),
            'test_size': len(trainer.X_test)
        }
        
        if PYTORCH_AVAILABLE:
            results['pytorch_accuracy'] = pytorch_acc
        if TENSORFLOW_AVAILABLE:
            results['tensorflow_accuracy'] = tf_acc
        
        results_path = os.path.join(trainer.models_path, 'training_results.json')
        with open(results_path, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"📊 Sonuçlar kaydedildi: {results_path}")
        
    except Exception as e:
        print(f"❌ Eğitim hatası: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
