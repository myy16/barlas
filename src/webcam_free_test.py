"""
Webcam-Free Tabela Test Sistemi
Kamera olmadan test yapılabilir, test görüntüleri yüklenebilir
"""
from flask import Flask, request, jsonify, render_template_string
import cv2
import numpy as np
from tabela_recognition import Tab        # Analiz et
        import time
        start_time = time.time()
        
        print(f"[DEBUG] Test görüntüsü analiz ediliyor - Rakam: {digit}")
        print(f"[DEBUG] Görüntü boyutu: {img.shape}")
        
        try:
            result = tabela_recognition.analyze_image(img, use_cnn=False)
            processing_time = f"{(time.time() - start_time):.2f}s"
            
            print(f"[DEBUG] Analiz sonucu: {result}")
            
            return jsonify({
                'success': True,
                'etap': result['etap'],
                'confidence': f"{result['confidence']:.3f}",
                'method': result['method'],
                'processing_time': processing_time,
                'debug': f'Test rakam: {digit}, Tanınan: {result["etap"]}'
            })
        except Exception as analyze_error:
            print(f"[HATA] Analiz hatası: {str(analyze_error)}")
            import traceback
            traceback.print_exc()
            return jsonify({
                'success': False, 
                'error': f'Analiz hatası: {str(analyze_error)}',
                'debug': f'Test rakam: {digit}, Hata detayı: {str(analyze_error)}'
            })
import base64
import io
from PIL import Image

app = Flask(__name__)

# Tabela tanıma sistemini başlat
tabela_recognition = TabelaRecognition()

# Test görüntüleri oluştur
def create_test_images():
    """Test için basit rakam görüntüleri oluştur"""
    test_images = {}
    
    for digit in [1, 2, 3, 4]:
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
        
        # Base64'e çevir
        _, buffer = cv2.imencode('.jpg', img)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        test_images[digit] = img_base64
    
    return test_images

# Test görüntülerini oluştur
test_images = create_test_images()

@app.route('/')
def index():
    """Ana sayfa - webcam-free test arayüzü"""
    return f'''
    <!DOCTYPE html>
    <html>
    <head>
        <title>BARLAS - Webcam-Free Tabela Test</title>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body {{ font-family: Arial, sans-serif; margin: 20px; text-align: center; }}
            .container {{ max-width: 900px; margin: 0 auto; }}
            .test-images {{ display: flex; justify-content: space-around; margin: 20px 0; }}
            .test-image {{ border: 2px solid #333; padding: 10px; margin: 10px; cursor: pointer; }}
            .test-image:hover {{ border-color: #007bff; }}
            .test-image.selected {{ border-color: #28a745; background: #e8f5e8; }}
            .controls {{ margin: 20px 0; }}
            button {{ padding: 10px 20px; margin: 10px; font-size: 16px; }}
            .result {{ margin: 20px 0; padding: 20px; border: 1px solid #ccc; background: #f9f9f9; }}
            .status {{ color: #007bff; font-weight: bold; }}
            .warning {{ color: #ff6b6b; }}
            .success {{ color: #4ecdc4; }}
            .upload-area {{ border: 2px dashed #ccc; padding: 30px; margin: 20px 0; }}
            .upload-area:hover {{ border-color: #007bff; }}
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🚗 BARLAS Tabela Tanıma Test Sistemi</h1>
            <p>Kamera olmadan tabela tanıma testleri</p>
            
            <h3>📋 Test Görüntüleri</h3>
            <div class="test-images">
                <div class="test-image" onclick="selectTestImage(1)">
                    <h4>Rakam 1</h4>
                    <img src="data:image/jpeg;base64,{test_images[1]}" width="150" height="150">
                </div>
                <div class="test-image" onclick="selectTestImage(2)">
                    <h4>Rakam 2</h4>
                    <img src="data:image/jpeg;base64,{test_images[2]}" width="150" height="150">
                </div>
                <div class="test-image" onclick="selectTestImage(3)">
                    <h4>Rakam 3</h4>
                    <img src="data:image/jpeg;base64,{test_images[3]}" width="150" height="150">
                </div>
                <div class="test-image" onclick="selectTestImage(4)">
                    <h4>Rakam 4</h4>
                    <img src="data:image/jpeg;base64,{test_images[4]}" width="150" height="150">
                </div>
            </div>
            
            <div class="controls">
                <button onclick="analyzeSelected()">🔍 Seçili Görüntüyü Analiz Et</button>
                <button onclick="clearResults()">🗑️ Sonuçları Temizle</button>
            </div>
            
            <h3>📁 Kendi Görüntünüzü Yükleyin</h3>
            <div class="upload-area" onclick="document.getElementById('fileInput').click()">
                <p>Tıklayın ve görüntü dosyası seçin</p>
                <input type="file" id="fileInput" accept="image/*" style="display:none;" onchange="uploadImage()">
            </div>
            
            <div id="result" class="result" style="display:none;">
                <h3>📊 Analiz Sonucu</h3>
                <div id="result-content"></div>
            </div>
            
            <div class="status">
                <p id="status">✅ Sistem hazır - Test görüntüsü seçin veya yükleyin</p>
            </div>
        </div>

        <script>
            let selectedImage = null;
            
            function selectTestImage(digit) {{
                selectedImage = digit;
                
                // Tüm seçimleri temizle
                document.querySelectorAll('.test-image').forEach(img => {{
                    img.classList.remove('selected');
                }});
                
                // Seçili olanı işaretle
                document.querySelectorAll('.test-image')[digit-1].classList.add('selected');
                
                document.getElementById('status').textContent = `Rakam ${{digit}} seçildi - Analiz etmek için butona tıklayın`;
            }}
            
            function analyzeSelected() {{
                if (!selectedImage) {{
                    alert('Önce bir test görüntüsü seçin!');
                    return;
                }}
                
                document.getElementById('status').textContent = '🔄 Analiz ediliyor...';
                
                fetch('/analyze_test_image', {{
                    method: 'POST',
                    headers: {{
                        'Content-Type': 'application/json',
                    }},
                    body: JSON.stringify({{digit: selectedImage}})
                }})
                .then(response => response.json())
                .then(data => {{
                    showResult(data);
                    document.getElementById('status').textContent = '✅ Analiz tamamlandı';
                }})
                .catch(error => {{
                    console.error('Error:', error);
                    document.getElementById('status').textContent = '❌ Analiz hatası!';
                }});
            }}
            
            function uploadImage() {{
                const fileInput = document.getElementById('fileInput');
                const file = fileInput.files[0];
                
                if (!file) return;
                
                const formData = new FormData();
                formData.append('image', file);
                
                document.getElementById('status').textContent = '🔄 Yüklenen görüntü analiz ediliyor...';
                
                fetch('/analyze_uploaded_image', {{
                    method: 'POST',
                    body: formData
                }})
                .then(response => response.json())
                .then(data => {{
                    showResult(data);
                    document.getElementById('status').textContent = '✅ Yüklenen görüntü analizi tamamlandı';
                }})
                .catch(error => {{
                    console.error('Error:', error);
                    document.getElementById('status').textContent = '❌ Yükleme hatası!';
                }});
            }}
            
            function showResult(data) {{
                const resultDiv = document.getElementById('result');
                const contentDiv = document.getElementById('result-content');
                
                if (data.success) {{
                    contentDiv.innerHTML = `
                        <p><strong>Tanınan Tabela:</strong> ${{data.etap}}</p>
                        <p><strong>Güven Skoru:</strong> ${{data.confidence}}</p>
                        <p><strong>Kullanılan Yöntem:</strong> ${{data.method}}</p>
                        <p><strong>İşlem Süresi:</strong> ${{data.processing_time}}</p>
                        <p><strong>Debug:</strong> ${{data.debug || 'Yok'}}</p>
                    `;
                    resultDiv.className = 'result success';
                }} else {{
                    contentDiv.innerHTML = `
                        <p class="warning"><strong>Hata:</strong> ${{data.error}}</p>
                    `;
                    resultDiv.className = 'result warning';
                }}
                
                resultDiv.style.display = 'block';
            }}
            
            function clearResults() {{
                document.getElementById('result').style.display = 'none';
                selectedImage = null;
                document.querySelectorAll('.test-image').forEach(img => {{
                    img.classList.remove('selected');
                }});
                document.getElementById('status').textContent = '✅ Sistem hazır - Test görüntüsü seçin veya yükleyin';
            }}
        </script>
    </body>
    </html>
    '''

@app.route('/analyze_test_image', methods=['POST'])
def analyze_test_image():
    """Test görüntüsünü analiz et"""
    try:
        data = request.get_json()
        digit = data.get('digit')
        
        if digit not in [1, 2, 3, 4]:
            return jsonify({{'success': False, 'error': 'Geçersiz rakam'}})
        
        # Test görüntüsünü decode et
        img_data = base64.b64decode(test_images[digit])
        img = cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)
        
        # Analiz et
        import time
        start_time = time.time()
        result = tabela_recognition.analyze_image(img, use_cnn=False)
        processing_time = f"{{(time.time() - start_time):.2f}}s"
        
        return jsonify({{
            'success': True,
            'etap': result['etap'],
            'confidence': f"{{result['confidence']:.3f}}",
            'method': result['method'],
            'processing_time': processing_time,
            'debug': f'Test rakam: {{digit}}, Tanınan: {{result["etap"]}}'
        }})
        
    except Exception as e:
        return jsonify({'success': False, 'error': f'Analiz hatası: {str(e)}'})

@app.route('/analyze_uploaded_image', methods=['POST'])
def analyze_uploaded_image():
    """Yüklenen görüntüyü analiz et"""
    try:
        if 'image' not in request.files:
            return jsonify({'success': False, 'error': 'Görüntü bulunamadı'})
        
        file = request.files['image']
        if file.filename == '':
            return jsonify({'success': False, 'error': 'Dosya seçilmedi'})
        
        # Görüntüyü oku
        img_stream = io.BytesIO(file.read())
        img_pil = Image.open(img_stream)
        img_array = np.array(img_pil)
        
        # PIL RGB -> OpenCV BGR dönüşümü
        if len(img_array.shape) == 3:
            img = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
        else:
            img = img_array
        
        # Analiz et
        import time
        start_time = time.time()
        result = tabela_recognition.analyze_image(img, use_cnn=False)
        processing_time = f"{(time.time() - start_time):.2f}s"
        
        return jsonify({
            'success': True,
            'etap': result['etap'],
            'confidence': f"{result['confidence']:.3f}",
            'method': result['method'],
            'processing_time': processing_time,
            'debug': f'Yüklenen dosya: {file.filename}'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': f'Yükleme hatası: {str(e)}'})

if __name__ == '__main__':
    print("🚗 BARLAS Webcam-Free Tabela Test Sistemi")
    print("✅ Kamera gerektirmez - Test görüntüleri veya dosya yükleme")
    print("🌐 http://localhost:5000 adresinde başlatılıyor...")
    app.run(debug=True, host='0.0.0.0', port=5000)