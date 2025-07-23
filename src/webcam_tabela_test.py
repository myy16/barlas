"""
Web Kamera ile Tabela Tanıma Test Sistemi
Bilgisayarın web kamerasıyla görüntüleri analiz eder
"""
from flask import Flask, request, jsonify, Response
import cv2
import numpy as np
from tabela_recognition import TabelaRecognition
import threading
import time

app = Flask(__name__)

# Tabela tanıma sistemini başlat
tabela_recognition = TabelaRecognition()

# Global değişkenler
current_frame = None
camera_running = False
cap = None

# Web kamera için konfigürasyon
def start_camera_thread():
    """Web kamerasını ayrı thread'de çalıştır"""
    global current_frame, camera_running, cap
    
    # DirectShow backend ile kamerayı aç (Windows için en stabil)
    print("DirectShow backend ile kamera açılıyor...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print("DirectShow başarısız, Media Foundation deneniyor...")
        cap = cv2.VideoCapture(0, cv2.CAP_MSMF)
        
        if not cap.isOpened():
            print("HATA: Hiçbir backend ile kamera açılamadı!")
            return
    
    # Kamera ayarları
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Buffer boyutunu küçült
    
    # Test frame
    ret, test_frame = cap.read()
    if not ret or test_frame is None:
        print("HATA: Test frame alınamadı!")
        cap.release()
        return
    
    camera_running = True
    print(f"Web kamerası başlatıldı! Frame boyutu: {test_frame.shape}")
    
    frame_count = 0
    error_count = 0
    while camera_running:
        ret, frame = cap.read()
        if ret and frame is not None:
            current_frame = frame.copy()
            frame_count += 1
            error_count = 0  # Başarılı frame, error sayacını sıfırla
            
            if frame_count % 100 == 0:  # Her 100 frame'de bir log
                print(f"Kamera aktif: {frame_count} frame alındı")
        else:
            error_count += 1
            if error_count > 10:  # Sürekli hata varsa yeniden başlat
                print("Çok fazla frame hatası, kamerayı yeniden başlatıyor...")
                cap.release()
                time.sleep(1)
                cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    error_count = 0
                else:
                    print("Kamera yeniden başlatılamadı!")
                    break
        
        time.sleep(0.033)  # ~30 FPS
    
    cap.release()
    print("Kamera kapatıldı")

def stop_camera():
    """Web kamerasını durdur"""
    global camera_running, cap
    camera_running = False
    if cap:
        cap.release()

# Kamera thread'ini başlat
camera_thread = threading.Thread(target=start_camera_thread, daemon=True)
camera_thread.start()

# Flask rotaları
@app.route('/')
def index():
    """Ana sayfa - web kamerası görüntüsü ve kontrol butonları"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>BARLAS - Tabela Tanıma Test</title>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; text-align: center; }
            .container { max-width: 800px; margin: 0 auto; }
            #video-container { border: 2px solid #333; margin: 20px 0; }
            #camera-feed { width: 100%; max-width: 640px; height: auto; }
            .controls { margin: 20px 0; }
            button { padding: 10px 20px; margin: 10px; font-size: 16px; }
            .result { margin: 20px 0; padding: 20px; border: 1px solid #ccc; background: #f9f9f9; }
            .status { color: #007bff; font-weight: bold; }
            .warning { color: #ff6b6b; }
            .success { color: #4ecdc4; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🚗 BARLAS Web Kamera Tabela Tanıma</h1>
            <p>Web kamerasından tabela görüntülerini analiz et</p>
            
            <div id="video-container">
                <img id="camera-feed" src="/video_feed" alt="Web Kamerası">
            </div>
            
            <div class="controls">
                <button onclick="analyzeFrame()">� Mevcut Görüntüyü Analiz Et</button>
                <button onclick="clearResults()">�️ Sonuçları Temizle</button>
                <br>
                <button onclick="showNumber(1)">1️⃣</button>
                <button onclick="showNumber(2)">2️⃣</button>
                <button onclick="showNumber(3)">3️⃣</button>
                <button onclick="showNumber(4)">4️⃣</button>
            </div>
            
            <div id="result" class="result" style="display:none;">
                <h3>📊 Analiz Sonucu</h3>
                <div id="result-content"></div>
            </div>
            
            <div class="status">
                <p id="status">📷 Web kamerası aktif - Tabela görüntülerini analiz edebilirsiniz</p>
            </div>
        </div>

        <script>
            function analyzeFrame() {
                document.getElementById('status').textContent = '🔄 Analiz ediliyor...';
                
                fetch('/analyze_current_frame', { method: 'POST' })
                    .then(response => response.json())
                    .then(data => {
                        showResult(data);
                        document.getElementById('status').textContent = '✅ Analiz tamamlandı';
                    })
                    .catch(error => {
                        console.error('Error:', error);
                        document.getElementById('status').textContent = '❌ Analiz hatası!';
                        document.getElementById('status').className = 'warning';
                    });
            }
            
            function showResult(data) {
                const resultDiv = document.getElementById('result');
                const contentDiv = document.getElementById('result-content');
                
                if (data.success) {
                    contentDiv.innerHTML = `
                        <p><strong>Tanınan Tabela:</strong> ${data.etap}</p>
                        <p><strong>Güven Skoru:</strong> ${data.confidence}</p>
                        <p><strong>Kullanılan Yöntem:</strong> ${data.method}</p>
                        <p><strong>İşlem Süresi:</strong> ${data.processing_time}</p>
                    `;
                    resultDiv.className = 'result success';
                } else {
                    contentDiv.innerHTML = `
                        <p class="warning"><strong>Hata:</strong> ${data.error}</p>
                    `;
                    resultDiv.className = 'result warning';
                }
                
                resultDiv.style.display = 'block';
            }
            
            function clearResults() {
                document.getElementById('result').style.display = 'none';
                document.getElementById('status').textContent = '📷 Web kamerası aktif - Tabela görüntülerini analiz edebilirsiniz';
                document.getElementById('status').className = 'status';
            }
            
            function showNumber(num) {
                // Test için büyük numara gösterici
                const modal = document.createElement('div');
                modal.style.cssText = `
                    position: fixed; top: 0; left: 0; width: 100%; height: 100%;
                    background: rgba(0,0,0,0.9); display: flex; justify-content: center;
                    align-items: center; z-index: 1000; font-size: 300px;
                    color: white; cursor: pointer; font-weight: bold;
                `;
                modal.textContent = num;
                modal.onclick = () => document.body.removeChild(modal);
                document.body.appendChild(modal);
                
                // 3 saniye sonra otomatik kapat
                setTimeout(() => {
                    if (document.body.contains(modal)) {
                        document.body.removeChild(modal);
                    }
                }, 3000);
            }
        </script>
    </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    """Web kamerası görüntü akışı"""
    def generate_frames():
        global current_frame
        while True:
            if current_frame is not None:
                # Frame'i JPEG formatına çevir
                ret, buffer = cv2.imencode('.jpg', current_frame)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.1)  # 10 FPS for web streaming
    
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/analyze_current_frame', methods=['POST'])
def analyze_current_frame():
    """Mevcut kamera frame'ini analiz et"""
    global current_frame
    
    if current_frame is None:
        return jsonify({
            'success': False,
            'error': 'Kamera görüntüsü bulunamadı'
        })
    
    try:
        start_time = time.time()
        
        # Görüntüyü analiz et
        result = tabela_recognition.analyze_image(current_frame, use_cnn=False)
        
        processing_time = f"{(time.time() - start_time):.2f}s"
        
        return jsonify({
            'success': True,
            'etap': result['etap'],
            'confidence': f"{result['confidence']:.3f}",
            'method': result['method'],
            'processing_time': processing_time
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': f'Analiz hatası: {str(e)}'
        })

@app.route('/status')
def status():
    """Sistem durumu"""
    return jsonify({
        'camera_running': camera_running,
        'camera_available': cap is not None and cap.isOpened() if cap else False,
        'tabela_system': 'Ready'
    })

if __name__ == '__main__':
    print("=== BARLAS Web Kamera Tabela Tanıma Sistemi ===")
    print("Sistem başlatılıyor...")
    time.sleep(1)  # Kameranın başlaması için kısa bekleme
    print(f"Web kamerası durumu: {'Aktif' if camera_running else 'Başlatılıyor'}")
    print("\nSistem hazır!")
    print("Web tarayıcınızdan şu adrese gidin:")
    print("http://localhost:5000")
    print("\nSistem durumu için:")
    print("http://localhost:5000/status")
    print("\nKullanım:")
    print("- Web kamerasından tabela görüntülerini analiz edebilirsiniz")
    print("- Test için 1-4 numaralarını ekranda gösterebilirsiniz")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    finally:
        # Uygulama kapanırken kamerayı temizle
        stop_camera()
