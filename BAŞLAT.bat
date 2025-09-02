@echo off
echo ============================================
echo  🚀 BARLAS SİSTEMİ HIZLI BAŞLATMA
echo ============================================
echo.

REM Gerekli dizine git
cd /d "d:\barlas\src\dart_laser_system"

echo 📂 Klasör: %cd%
echo.

REM Python kurulu mu kontrol et
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Python bulunamadı! Lütfen Python'u kurun.
    pause
    exit /b 1
)

echo ✅ Python kontrol edildi
echo.

REM Gerekli dosyalar var mı kontrol et
if not exist "arduino_pantilt_fixed.ino" (
    echo ❌ Arduino kodu bulunamadı!
    pause
    exit /b 1
)

if not exist "yolo_arduino_dart_system.py" (
    echo ❌ Ana sistem dosyası bulunamadı!
    pause
    exit /b 1
)

if not exist "best.onnx" (
    echo ⚠️  YOLO model dosyası bulunamadı! 
    echo    Sistem çalışmayabilir.
    echo.
)

echo ✅ Dosyalar kontrol edildi
echo.

REM Kullanıcı seçimi
echo Hangi işlemi yapmak istiyorsunuz?
echo.
echo 1. 🧪 Sistem Testi (Önce bunu yapın!)
echo 2. 🚀 Ana Sistemi Çalıştır
echo 3. ❌ İptal
echo.
set /p choice=Seçiminiz (1/2/3): 

if "%choice%"=="1" goto test
if "%choice%"=="2" goto main
if "%choice%"=="3" goto end
goto end

:test
echo.
echo 🧪 Sistem testi başlatılıyor...
echo.
python test_system_integration.py
goto end

:main
echo.
echo 🚀 Ana sistem başlatılıyor...
echo.
echo 💡 İpucu: 'q' tuşu ile çıkış yapabilirsiniz
echo.
python yolo_arduino_dart_system.py
goto end

:end
echo.
echo ✅ İşlem tamamlandı.
pause
