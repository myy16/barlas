# BARLAS Arduino Dart Targeting System - Kurulum Script
# Windows PowerShell için otomatik kurulum scripti

Write-Host "🎯 BARLAS Arduino Dart System Kurulumu Başlıyor..." -ForegroundColor Green
Write-Host "=" * 60

# 1. Python Versiyonu Kontrolü
Write-Host "🐍 Python versiyonu kontrol ediliyor..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    if ($pythonVersion -match "Python (\d+)\.(\d+)") {
        $majorVersion = [int]$matches[1]
        $minorVersion = [int]$matches[2]
        
        if ($majorVersion -ge 3 -and $minorVersion -ge 8) {
            Write-Host "✅ Python $pythonVersion - OK" -ForegroundColor Green
        } else {
            Write-Host "❌ Python 3.8+ gerekli! Mevcut: $pythonVersion" -ForegroundColor Red
            exit 1
        }
    }
} catch {
    Write-Host "❌ Python bulunamadı! Lütfen Python 3.8+ yükleyin" -ForegroundColor Red
    Write-Host "İndirme: https://www.python.org/downloads/" -ForegroundColor Blue
    exit 1
}

# 2. Pip Güncelleme
Write-Host "📦 Pip güncelleniyor..." -ForegroundColor Yellow
python -m pip install --upgrade pip

# 3. Gerekli Kütüphaneler
Write-Host "📚 Python kütüphaneleri yükleniyor..." -ForegroundColor Yellow

$packages = @(
    "opencv-python>=4.8.0",
    "numpy>=1.21.0", 
    "PyYAML>=6.0",
    "pyserial>=3.5"
)

foreach ($package in $packages) {
    Write-Host "  Yükleniyor: $package" -ForegroundColor Cyan
    python -m pip install $package
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  ✅ $package - OK" -ForegroundColor Green
    } else {
        Write-Host "  ❌ $package - HATA" -ForegroundColor Red
    }
}

# 4. Opsiyonel Kütüphaneler
Write-Host "🎨 Opsiyonel kütüphaneler yükleniyor..." -ForegroundColor Yellow

$optionalPackages = @(
    "onnxruntime",
    "matplotlib", 
    "pillow"
)

foreach ($package in $optionalPackages) {
    Write-Host "  Yükleniyor: $package" -ForegroundColor Cyan
    python -m pip install $package 2>$null
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  ✅ $package - OK" -ForegroundColor Green
    } else {
        Write-Host "  ⚠️ $package - Atlandı (opsiyonel)" -ForegroundColor Yellow
    }
}

# 5. Import Testleri
Write-Host "🧪 Import testleri yapılıyor..." -ForegroundColor Yellow

$imports = @(
    @("cv2", "OpenCV"),
    @("numpy", "NumPy"),
    @("serial", "PySerial"),
    @("yaml", "PyYAML")
)

foreach ($import in $imports) {
    $module = $import[0]
    $name = $import[1]
    
    $result = python -c "try: import $module; print('OK')" 2>$null
    
    if ($result -eq "OK") {
        Write-Host "  ✅ $name - OK" -ForegroundColor Green
    } else {
        Write-Host "  ❌ $name - HATA" -ForegroundColor Red
    }
}

# 6. Kamera Kontrolü
Write-Host "📹 Kamera kontrolü yapılıyor..." -ForegroundColor Yellow

$cameraTest = python -c @"
import cv2
found_cameras = []
for i in range(3):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            found_cameras.append(f'{i}: {w}x{h}')
    cap.release()

if found_cameras:
    print('CAMERAS_FOUND:' + '|'.join(found_cameras))
else:
    print('NO_CAMERAS')
"@

if ($cameraTest -match "CAMERAS_FOUND:(.+)") {
    $cameras = $matches[1] -split '\|'
    Write-Host "  ✅ Bulunan kameralar:" -ForegroundColor Green
    foreach ($camera in $cameras) {
        Write-Host "    $camera" -ForegroundColor Cyan
    }
} else {
    Write-Host "  ⚠️ Kamera bulunamadı" -ForegroundColor Yellow
}

# 7. Sistem Dosyaları Kontrolü
Write-Host "📂 Sistem dosyaları kontrol ediliyor..." -ForegroundColor Yellow

$requiredFiles = @(
    "arduino_controller.py",
    "arduino_pantilt.ino", 
    "yolo_arduino_dart_system.py",
    "camera_test.py",
    "requirements.txt"
)

$missingFiles = @()

foreach ($file in $requiredFiles) {
    if (Test-Path $file) {
        Write-Host "  ✅ $file - OK" -ForegroundColor Green
    } else {
        Write-Host "  ❌ $file - EKSIK" -ForegroundColor Red
        $missingFiles += $file
    }
}

if ($missingFiles.Count -eq 0) {
    Write-Host "  ✅ Tüm dosyalar mevcut" -ForegroundColor Green
} else {
    Write-Host "  ⚠️ Eksik dosyalar var - Git clone kontrol edin" -ForegroundColor Yellow
}

# 8. Arduino Test (Opsiyonel)
Write-Host "🤖 Arduino bağlantısı kontrol ediliyor..." -ForegroundColor Yellow

$comPorts = python -c @"
import serial.tools.list_ports
ports = []
for port in serial.tools.list_ports.comports():
    if 'Arduino' in port.description or 'CH340' in port.description or 'USB' in port.description:
        ports.append(f'{port.device} - {port.description}')

if ports:
    print('ARDUINO_PORTS:' + '|'.join(ports))
else:
    print('NO_ARDUINO')
"@

if ($comPorts -match "ARDUINO_PORTS:(.+)") {
    $ports = $matches[1] -split '\|'
    Write-Host "  ✅ Arduino portları bulundu:" -ForegroundColor Green
    foreach ($port in $ports) {
        Write-Host "    $port" -ForegroundColor Cyan
    }
} else {
    Write-Host "  ⚠️ Arduino bulunamadı (USB bağlı değil veya driver eksik)" -ForegroundColor Yellow
}

# 9. Kurulum Özeti
Write-Host "📋 KURULUM ÖZETİ" -ForegroundColor Magenta
Write-Host "=" * 60

# Sistem durumu kontrol et
$systemReady = $true

# Python kontrol
try {
    python -c "import cv2, numpy, serial, yaml" 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Python Kütüphaneleri: HAZIR" -ForegroundColor Green
    } else {
        Write-Host "❌ Python Kütüphaneleri: EKSIK" -ForegroundColor Red  
        $systemReady = $false
    }
} catch {
    Write-Host "❌ Python Kütüphaneleri: HATA" -ForegroundColor Red
    $systemReady = $false
}

# YOLO kontrol
if (Test-Path "..\dart_recognize\yolo_predictions.py") {
    Write-Host "✅ YOLO Modülü: MEVCUT" -ForegroundColor Green
} else {
    Write-Host "⚠️ YOLO Modülü: KONTROL EDİN" -ForegroundColor Yellow
}

# Dosya kontrolü
if ($missingFiles.Count -eq 0) {
    Write-Host "✅ Sistem Dosyaları: TAMAM" -ForegroundColor Green
} else {
    Write-Host "❌ Sistem Dosyaları: EKSIK" -ForegroundColor Red
    $systemReady = $false
}

Write-Host ""

if ($systemReady) {
    Write-Host "🎉 SİSTEM HAZIR!" -ForegroundColor Green
    Write-Host ""
    Write-Host "🚀 Başlatma Komutları:" -ForegroundColor Cyan
    Write-Host "  cd src\dart_laser_system" -ForegroundColor White
    Write-Host "  python yolo_arduino_dart_system.py" -ForegroundColor White
    Write-Host ""
    Write-Host "📋 Test Komutları:" -ForegroundColor Cyan  
    Write-Host "  python camera_test.py" -ForegroundColor White
    Write-Host "  python yolo_arduino_dart_system.py --list-cameras" -ForegroundColor White
} else {
    Write-Host "⚠️ SİSTEM HAZIR DEĞİL!" -ForegroundColor Red
    Write-Host "Lütfen eksik kütüphaneleri ve dosyaları kontrol edin" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "📖 Detaylı bilgi için INSTALLATION_GUIDE.md dosyasını okuyun" -ForegroundColor Blue
Write-Host "🎯 BARLAS Arduino Dart Targeting System" -ForegroundColor Magenta
