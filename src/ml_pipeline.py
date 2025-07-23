"""
BARLAS Tabela ML Pipeline - Tam İş Akışı
1. Veri Toplama
2. Model Eğitimi  
3. Model Testi
"""
import os
import sys

def show_menu():
    """Ana menüyü göster"""
    print("\n" + "="*60)
    print("🚗 BARLAS TABELA MAKİNE ÖĞRENMESİ PİPELİNE")
    print("="*60)
    print("1. 📸 Veri Toplama (Kameradan görüntü kaydetme)")
    print("2. 🤖 Model Eğitimi (CNN ile eğitim)")
    print("3. 🎯 Model Testi (Eğitilmiş modeli test et)")
    print("4. 📊 Veri Seti İstatistikleri")
    print("5. 🚗 Canlı Tabela Tanıma (Kamera ile)")
    print("6. 🌐 Web Interface (Tarayıcı testi)")
    print("0. ❌ Çıkış")
    print("="*60)

def check_dataset():
    """Veri seti durumunu kontrol et"""
    dataset_path = "d:/barlas/dataset/labeled_data"
    
    if not os.path.exists(dataset_path):
        return False, "Veri seti klasörü bulunamadı"
    
    counts = {}
    total = 0
    
    for digit in [1, 2, 3, 4]:
        digit_path = os.path.join(dataset_path, f"digit_{digit}")
        if os.path.exists(digit_path):
            count = len([f for f in os.listdir(digit_path) if f.endswith('.jpg')])
            counts[f"digit_{digit}"] = count
            total += count
        else:
            counts[f"digit_{digit}"] = 0
    
    return True, f"Toplam {total} etiketli görüntü: {counts}"

def check_models():
    """Model durumunu kontrol et"""
    models_path = "d:/barlas/dataset/models"
    
    if not os.path.exists(models_path):
        return False, "Model klasörü bulunamadı"
    
    import glob
    pytorch_models = glob.glob(os.path.join(models_path, "pytorch_*.pth"))
    tf_models = glob.glob(os.path.join(models_path, "tensorflow_*.h5"))
    
    if not pytorch_models and not tf_models:
        return False, "Hiç eğitilmiş model bulunamadı"
    
    return True, f"PyTorch: {len(pytorch_models)}, TensorFlow: {len(tf_models)} model"

def run_data_collection():
    """Veri toplama başlat"""
    print("\n🚀 Veri toplama başlatılıyor...")
    print("💡 İpucu: Telefonunuzda 1-4 rakamlarını büyük fontla gösterin")
    print("💡 Her rakam için en az 50-100 örnek toplayın")
    
    try:
        from data_collector import main as data_main
        data_main()
    except Exception as e:
        print(f"❌ Hata: {e}")

def run_model_training():
    """Model eğitimi başlat"""
    exists, status = check_dataset()
    
    if not exists:
        print(f"❌ {status}")
        print("💡 Önce veri toplama yapın (Seçenek 1)")
        return
    
    print(f"✅ {status}")
    
    # Minimum veri kontrolü
    dataset_path = "d:/barlas/dataset/labeled_data"
    total_images = 0
    
    for digit in [1, 2, 3, 4]:
        digit_path = os.path.join(dataset_path, f"digit_{digit}")
        if os.path.exists(digit_path):
            count = len([f for f in os.listdir(digit_path) if f.endswith('.jpg')])
            total_images += count
    
    if total_images < 40:  # Minimum 10 per class
        print(f"⚠️ Çok az veri var ({total_images} görüntü)")
        print("💡 En az 40 görüntü (her rakam için 10) toplayın")
        return
    
    print(f"\n🚀 Model eğitimi başlatılıyor...")
    print(f"📊 Toplam veri: {total_images} görüntü")
    
    try:
        from model_trainer import main as train_main
        train_main()
    except Exception as e:
        print(f"❌ Hata: {e}")
        import traceback
        traceback.print_exc()

def run_model_test():
    """Model testi başlat"""
    exists, status = check_models()
    
    if not exists:
        print(f"❌ {status}")
        print("💡 Önce model eğitimi yapın (Seçenek 2)")
        return
    
    print(f"✅ {status}")
    
    try:
        from trained_digit_recognizer import test_trained_model
        test_trained_model()
    except Exception as e:
        print(f"❌ Hata: {e}")

def show_dataset_stats():
    """Veri seti istatistiklerini göster"""
    dataset_path = "d:/barlas/dataset"
    
    print("\n📊 VERİ SETİ İSTATİSTİKLERİ")
    print("="*40)
    
    # Etiketli veri
    labeled_path = os.path.join(dataset_path, "labeled_data")
    if os.path.exists(labeled_path):
        total_labeled = 0
        for digit in [1, 2, 3, 4]:
            digit_path = os.path.join(labeled_path, f"digit_{digit}")
            if os.path.exists(digit_path):
                count = len([f for f in os.listdir(digit_path) if f.endswith('.jpg')])
                total_labeled += count
                progress = min(100, (count / 50) * 100)  # 50 per class target
                print(f"Rakam {digit}: {count:4d} örnek ({progress:5.1f}%)")
        
        print(f"Toplam etiketli: {total_labeled:4d} örnek")
    
    # Ham veri
    raw_path = os.path.join(dataset_path, "raw_images")
    if os.path.exists(raw_path):
        raw_count = len([f for f in os.listdir(raw_path) if f.endswith('.jpg')])
        print(f"Ham görüntü: {raw_count:4d} örnek")
    
    # Modeller
    models_path = os.path.join(dataset_path, "models")
    if os.path.exists(models_path):
        import glob
        pytorch_count = len(glob.glob(os.path.join(models_path, "pytorch_*.pth")))
        tf_count = len(glob.glob(os.path.join(models_path, "tensorflow_*.h5")))
        print(f"PyTorch modeller: {pytorch_count}")
        print(f"TensorFlow modeller: {tf_count}")
        
        # Eğitim sonuçları
        results_file = os.path.join(models_path, 'training_results.json')
        if os.path.exists(results_file):
            try:
                import json
                with open(results_file, 'r') as f:
                    results = json.load(f)
                
                print(f"\n🎯 En son eğitim sonuçları:")
                if 'pytorch_accuracy' in results:
                    print(f"PyTorch accuracy: {results['pytorch_accuracy']:.2f}%")
                if 'tensorflow_accuracy' in results:
                    print(f"TensorFlow accuracy: {results['tensorflow_accuracy']:.2f}%")
            except:
                pass
    
    print("="*40)

def run_live_recognition():
    """Canlı tabela tanıma başlat"""
    print("\n🚀 Canlı tabela tanıma başlatılıyor...")
    print("💡 İpucu: SPACE ile analiz, R ile sürekli mod")
    
    try:
        from camera_tabela_live import main as live_main
        live_main()
    except Exception as e:
        print(f"❌ Hata: {e}")

def run_web_interface():
    """Web interface başlat"""
    print("\n🌐 Web interface başlatılıyor...")
    print("🔗 http://localhost:5000 adresinde açılacak")
    
    try:
        import subprocess
        import time
        
        # Web server'ı background'da başlat
        subprocess.Popen([
            sys.executable, 
            "d:/barlas/src/webcam_free_test_fixed.py"
        ], 
        cwd="d:/barlas/src")
        
        time.sleep(2)
        print("✅ Web server başlatıldı!")
        print("🌐 Tarayıcınızda http://localhost:5000 adresini açın")
        
    except Exception as e:
        print(f"❌ Hata: {e}")

def main():
    """Ana program"""
    while True:
        show_menu()
        
        try:
            choice = input("\nSeçiminizi yapın (0-6): ").strip()
            
            if choice == '0':
                print("👋 Çıkış yapılıyor...")
                break
            elif choice == '1':
                run_data_collection()
            elif choice == '2':
                run_model_training()
            elif choice == '3':
                run_model_test()
            elif choice == '4':
                show_dataset_stats()
            elif choice == '5':
                run_live_recognition()
            elif choice == '6':
                run_web_interface()
            else:
                print("❌ Geçersiz seçim!")
                
        except KeyboardInterrupt:
            print("\n👋 Program sonlandırıldı")
            break
        except Exception as e:
            print(f"❌ Hata: {e}")

if __name__ == "__main__":
    main()
