@echo off
echo ============================================
echo  🚗 BARLAS ARAÇ KONTROL SİSTEMİ
echo ============================================
echo.

REM Gerekli dizine git
cd /d "d:\barlas\src\dart_laser_system"

echo 📂 Klasör: %cd%
echo.

REM Güvenlik uyarısı
echo ⚠️  GÜVENLİK UYARISI:
echo    - Sadece güvenli, kapalı alanlarda kullanın!
echo    - Acil durdurma butonunu hazır tutun!
echo    - Fren sisteminin çalıştığından emin olun!
echo.
set /p safety=Bu güvenlik kurallarını kabul ediyor musunuz? (y/N): 

if /i not "%safety%"=="y" (
    echo ❌ Güvenlik kuralları kabul edilmedi. Çıkış yapılıyor.
    pause
    exit /b 1
)

echo.
echo ✅ Güvenlik kuralları kabul edildi.
echo.

REM İşlem seçimi
echo Hangi modu çalıştırmak istiyorsunz?
echo.
echo 1. 🧪 Sistem Testi (Önce bunu yapın!)
echo 2. 🚗 Güvenli Araç Test Modu
echo 3. 🎯 YOLO + Araç Entegrasyonu
echo 4. 🔧 Sistem Durumu Kontrolü
echo 5. ❌ İptal
echo.
set /p choice=Seçiminiz (1/2/3/4/5): 

if "%choice%"=="1" goto test
if "%choice%"=="2" goto vehicle_test
if "%choice%"=="3" goto yolo_vehicle
if "%choice%"=="4" goto system_check
if "%choice%"=="5" goto end
goto end

:test
echo.
echo 🧪 Sistem testi başlatılıyor...
echo 💡 Arduino bağlantısını kontrol edin!
echo.
python test_system_integration.py
goto end

:vehicle_test
echo.
echo 🚗 Güvenli araç test modu başlatılıyor...
echo.
echo 🎮 Kontroller:
echo    W/S     - İleri/Geri
echo    SPACE   - Dur
echo    B       - Fren
echo    F1      - ACİL FREN
echo    ESC     - Çıkış
echo.
python araç_test_modu.py
goto end

:yolo_vehicle
echo.
echo 🎯 YOLO + Araç entegrasyonu başlatılıyor...
echo 📹 Kameranızın bağlı olduğundan emin olun!
echo.
python yolo_arduino_dart_system.py
goto end

:system_check
echo.
echo 🔧 Sistem durumu kontrol ediliyor...
echo.
python sistem_kontrol.py
goto end

:end
echo.
echo ✅ İşlem tamamlandı.
echo 🛡️  Güvenlik: Sistemi kapatmadan önce aracın durduğundan emin olun!
pause
