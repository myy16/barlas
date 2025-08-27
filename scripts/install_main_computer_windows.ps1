# BARLAS Ana Bilgisayar PowerShell Kurulum Scripti
# Windows 11 + WSL2 Hazırlık ve Network Konfigürasyonu

param(
    [switch]$InstallWSL,
    [switch]$ConfigureNetwork,
    [switch]$InstallMissionPlanner,
    [switch]$All
)

# Admin kontrolü
if (-NOT ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    Write-Host "❌ Bu script yönetici olarak çalıştırılmalı!" -ForegroundColor Red
    Write-Host "PowerShell'i 'Yönetici olarak çalıştır' ile açın." -ForegroundColor Yellow
    exit 1
}

Write-Host "🎯 BARLAS Ana Bilgisayar Windows Kurulumu" -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan

# === WSL2 Kurulumu ===
if ($InstallWSL -or $All) {
    Write-Host "🔄 WSL2 kurulumu kontrol ediliyor..." -ForegroundColor Blue
    
    # WSL feature'ını etkinleştir
    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
    dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
    
    # WSL2'yi varsayılan olarak ayarla
    wsl --set-default-version 2
    
    # Ubuntu 20.04 kurulumu (eğer yüklü değilse)
    if (-not (wsl -l | Select-String "Ubuntu-20.04")) {
        Write-Host "📦 Ubuntu 20.04 kuruluyor..." -ForegroundColor Yellow
        wsl --install -d Ubuntu-20.04
    } else {
        Write-Host "✅ Ubuntu 20.04 zaten kurulu" -ForegroundColor Green
    }
    
    Write-Host "✅ WSL2 kurulumu tamamlandı" -ForegroundColor Green
}

# === Network Konfigürasyonu ===
if ($ConfigureNetwork -or $All) {
    Write-Host "🌐 Network port forwarding konfigürasyonu..." -ForegroundColor Blue
    
    # Mevcut port forwarding kurallarını temizle
    netsh interface portproxy reset
    
    # WSL2 IP adresini al
    $wslIP = wsl hostname -I
    $wslIP = $wslIP.Trim()
    
    if ($wslIP) {
        Write-Host "🔍 WSL2 IP adresi: $wslIP" -ForegroundColor Yellow
        
        # Port forwarding kuralları ekle
        Write-Host "📡 Port forwarding kuralları ekleniyor..." -ForegroundColor Blue
        
        # ArduPilot SITL portu
        netsh interface portproxy add v4tov4 listenport=5760 listenaddress=0.0.0.0 connectport=5760 connectaddress=$wslIP
        
        # Mission Planner telemetry portu
        netsh interface portproxy add v4tov4 listenport=14550 listenaddress=0.0.0.0 connectport=14550 connectaddress=$wslIP
        
        # ROS Master portu
        netsh interface portproxy add v4tov4 listenport=11311 listenaddress=0.0.0.0 connectport=11311 connectaddress=$wslIP
        
        # Firewall kuralları ekle
        Write-Host "🔒 Firewall kuralları ekleniyor..." -ForegroundColor Blue
        
        New-NetFirewallRule -DisplayName "WSL2 ArduPilot SITL" -Direction Inbound -LocalPort 5760 -Protocol TCP -Action Allow
        New-NetFirewallRule -DisplayName "WSL2 Mission Planner" -Direction Inbound -LocalPort 14550 -Protocol UDP -Action Allow
        New-NetFirewallRule -DisplayName "WSL2 ROS Master" -Direction Inbound -LocalPort 11311 -Protocol TCP -Action Allow
        
        Write-Host "✅ Network konfigürasyonu tamamlandı" -ForegroundColor Green
        
        # Port forwarding listesini göster
        Write-Host "📋 Aktif port forwarding kuralları:" -ForegroundColor Yellow
        netsh interface portproxy show all
        
    } else {
        Write-Host "❌ WSL2 IP adresi alınamadı! WSL2'nin çalıştığından emin olun." -ForegroundColor Red
    }
}

# === Mission Planner Kurulumu ===
if ($InstallMissionPlanner -or $All) {
    Write-Host "🚁 Mission Planner kurulumu..." -ForegroundColor Blue
    
    $missionPlannerPath = "C:\Program Files (x86)\Mission Planner\MissionPlanner.exe"
    
    if (-not (Test-Path $missionPlannerPath)) {
        Write-Host "📦 Mission Planner indiriliyor..." -ForegroundColor Yellow
        
        # Mission Planner'ı indir
        $downloadUrl = "https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi"
        $downloadPath = "$env:TEMP\MissionPlanner-latest.msi"
        
        try {
            Invoke-WebRequest -Uri $downloadUrl -OutFile $downloadPath
            Write-Host "📦 Mission Planner kurulumu başlatılıyor..." -ForegroundColor Yellow
            Start-Process -FilePath $downloadPath -Wait
            
            if (Test-Path $missionPlannerPath) {
                Write-Host "✅ Mission Planner kurulumu tamamlandı" -ForegroundColor Green
            } else {
                Write-Host "⚠️  Mission Planner kurulumu doğrulanamadı" -ForegroundColor Yellow
            }
        } catch {
            Write-Host "❌ Mission Planner indirme hatası: $($_.Exception.Message)" -ForegroundColor Red
        }
    } else {
        Write-Host "✅ Mission Planner zaten kurulu" -ForegroundColor Green
    }
}

# === Sistem Kontrolleri ===
Write-Host "`n🔍 Sistem Durumu Kontrolleri:" -ForegroundColor Cyan

# WSL2 durumu
Write-Host "WSL2 Durumu:" -ForegroundColor Yellow
wsl -l -v

# Network durumu
Write-Host "`nPort Forwarding Durumu:" -ForegroundColor Yellow
netsh interface portproxy show all

# Firewall durumu
Write-Host "`nFirewall Kuralları:" -ForegroundColor Yellow
Get-NetFirewallRule -DisplayName "*WSL2*" | Select-Object DisplayName, Enabled, Direction

Write-Host "`n🎉 BARLAS Windows Kurulumu Tamamlandı!" -ForegroundColor Green
Write-Host "=============================================" -ForegroundColor Green

Write-Host "`n📋 Sonraki Adımlar:" -ForegroundColor Cyan
Write-Host "1. WSL2 Ubuntu'yu başlatın: wsl" -ForegroundColor White
Write-Host "2. BARLAS Linux kurulum scriptini çalıştırın:" -ForegroundColor White
Write-Host "   chmod +x install_main_computer.sh" -ForegroundColor Gray
Write-Host "   ./install_main_computer.sh" -ForegroundColor Gray
Write-Host "3. Mission Planner'ı başlatın ve UDP:14550'ye bağlanın" -ForegroundColor White

Write-Host "`n🔧 Test Komutları:" -ForegroundColor Yellow
Write-Host "# WSL2'de ArduPilot SITL test:" -ForegroundColor Gray
Write-Host "cd ~/ardupilot && python3 Tools/autotest/sim_vehicle.py -v Rover --out=udp:0.0.0.0:14550" -ForegroundColor Gray
Write-Host "`n# Mission Planner bağlantı:" -ForegroundColor Gray
Write-Host "Connection Type: UDP, Port: 14550, Connect" -ForegroundColor Gray
