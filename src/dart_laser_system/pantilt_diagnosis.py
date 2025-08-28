#!/usr/bin/env python3
"""
BARLAS Pan-Tilt Diagnosis Tool
Arduino servo hareket problemi teşhis ve çözüm aracı
Ana bilgisayarda çalıştırılacak
"""

import serial
import time
import sys
import glob
from typing import List, Optional

class PanTiltDiagnosis:
    """Pan-Tilt sistem teşhis aracı"""
    
    def __init__(self):
        self.serial_conn = None
        self.port = None
        self.baud_rate = 9600
        
    def find_all_ports(self) -> List[str]:
        """Tüm mevcut serial portları bul"""
        ports = []
        
        if sys.platform.startswith('win'):
            # Windows COM portları
            for i in range(1, 21):
                ports.append(f'COM{i}')
        else:
            # Linux/Mac USB portları
            ports.extend(glob.glob('/dev/ttyUSB*'))
            ports.extend(glob.glob('/dev/ttyACM*'))
            ports.extend(glob.glob('/dev/cu.usb*'))
        
        return ports
    
    def test_port(self, port: str, baud: int = 9600) -> bool:
        """Belirli portu test et"""
        try:
            print(f"🔍 {port} portu test ediliyor (baud: {baud})...")
            
            # Serial bağlantı
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=3,
                write_timeout=3
            )
            
            # Arduino reset süresini bekle
            time.sleep(3)
            
            # Buffer temizle
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Test komutları
            test_commands = ["TEST\\n", "STATUS\\n", "PING\\n"]
            
            for cmd in test_commands:
                ser.write(cmd.encode())
                ser.flush()
                time.sleep(0.5)
                
                # Yanıt kontrol et
                response = ""
                start_time = time.time()
                
                while (time.time() - start_time) < 2:
                    if ser.in_waiting > 0:
                        response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    time.sleep(0.1)
                
                if response:
                    print(f"📡 Yanıt alındı: {response.strip()}")
                    
                    # Arduino'dan gelme kontrol
                    if any(word in response.upper() for word in ["OK", "READY", "BARLAS", "ERROR"]):
                        ser.close()
                        print(f"✅ {port} portunda Arduino bulundu!")
                        return True
            
            ser.close()
            print(f"❌ {port} portunda Arduino yok")
            return False
            
        except Exception as e:
            print(f"❌ {port} port hatası: {e}")
            return False
    
    def scan_all_ports(self) -> Optional[str]:
        """Tüm portları tara ve Arduino'yu bul"""
        print("🔎 Arduino aranıyor...")
        print("=" * 50)
        
        ports = self.find_all_ports()
        
        if not ports:
            print("❌ Hiç serial port bulunamadı!")
            return None
        
        for port in ports:
            if self.test_port(port):
                return port
        
        print("❌ Hiçbir portta Arduino bulunamadı!")
        return None
    
    def connect_to_arduino(self, port: str) -> bool:
        """Arduino'ya bağlan"""
        try:
            self.port = port
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=self.baud_rate,
                timeout=2
            )
            
            time.sleep(3)  # Arduino başlatma süresi
            
            # Bağlantı test et
            self.serial_conn.write("TEST\\n".encode())
            self.serial_conn.flush()
            time.sleep(0.5)
            
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode().strip()
                if "OK" in response or "READY" in response:
                    print(f"✅ Arduino bağlantısı başarılı: {port}")
                    return True
            
            return False
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def send_command(self, command: str) -> str:
        """Arduino'ya komut gönder ve yanıt al"""
        if not self.serial_conn:
            return "ERROR: Not connected"
        
        try:
            # Buffer temizle
            self.serial_conn.reset_input_buffer()
            
            # Komut gönder
            self.serial_conn.write((command + "\\n").encode())
            self.serial_conn.flush()
            
            # Yanıt bekle
            time.sleep(0.3)
            response = ""
            
            start_time = time.time()
            while (time.time() - start_time) < 2:
                if self.serial_conn.in_waiting > 0:
                    response += self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                time.sleep(0.1)
            
            return response.strip()
            
        except Exception as e:
            return f"ERROR: {e}"
    
    def test_servo_movement(self) -> bool:
        """Servo hareket testi"""
        print("\\n🎯 SERVO HAREKET TESTİ BAŞLATILIYOR...")
        print("=" * 50)
        
        test_positions = [
            ("Merkez", "MOVE,90,90"),
            ("Sol", "MOVE,60,90"),
            ("Sağ", "MOVE,120,90"),
            ("Yukarı", "MOVE,90,120"),
            ("Aşağı", "MOVE,90,60"),
            ("Sol Üst", "MOVE,60,120"),
            ("Sağ Alt", "MOVE,120,60"),
            ("Merkez", "CENTER")
        ]
        
        success_count = 0
        
        for name, command in test_positions:
            print(f"\\n📍 {name} pozisyonu: {command}")
            response = self.send_command(command)
            
            if "OK" in response:
                print(f"   ✅ Başarılı: {response}")
                success_count += 1
            else:
                print(f"   ❌ Başarısız: {response}")
            
            # Servo hareket süresini bekle
            time.sleep(1.5)
            
            # Durumu sorgula
            status = self.send_command("STATUS")
            print(f"   📊 Durum: {status}")
        
        print(f"\\n📊 Test Sonucu: {success_count}/{len(test_positions)} başarılı")
        return success_count >= len(test_positions) * 0.7  # %70 başarı
    
    def test_laser(self) -> bool:
        """Lazer testi"""
        print("\\n🔴 LAZER TESTİ BAŞLATILIYOR...")
        print("=" * 30)
        
        # Lazer aç
        print("🔴 Lazer açılıyor...")
        response = self.send_command("LASER,ON")
        print(f"   Yanıt: {response}")
        
        if "OK" in response:
            print("   ✅ Lazer açıldı")
            time.sleep(2)
            
            # Lazer kapat
            print("⚫ Lazer kapatılıyor...")
            response = self.send_command("LASER,OFF")
            print(f"   Yanıt: {response}")
            
            if "OK" in response:
                print("   ✅ Lazer kapatıldı")
                return True
            else:
                print("   ❌ Lazer kapatma başarısız")
                return False
        else:
            print("   ❌ Lazer açma başarısız")
            return False
    
    def interactive_control(self):
        """İnteraktif servo kontrolü"""
        print("\\n🎮 İNTERAKTİF KONTROL MODU")
        print("=" * 40)
        print("Komutlar:")
        print("  w/s: Tilt yukarı/aşağı")
        print("  a/d: Pan sol/sağ")
        print("  c: Merkez pozisyon")
        print("  l: Lazer aç/kapat")
        print("  t: Status göster")
        print("  q: Çıkış")
        print("-" * 40)
        
        current_pan = 90
        current_tilt = 90
        laser_on = False
        step_size = 5
        
        while True:
            try:
                command = input("\\nKomut (w/a/s/d/c/l/t/q): ").strip().lower()
                
                if command == 'q':
                    break
                elif command == 'w':  # Yukarı
                    current_tilt = min(150, current_tilt + step_size)
                    response = self.send_command(f"TILT,{current_tilt}")
                    print(f"Yukarı: Tilt={current_tilt}° | {response}")
                elif command == 's':  # Aşağı
                    current_tilt = max(30, current_tilt - step_size)
                    response = self.send_command(f"TILT,{current_tilt}")
                    print(f"Aşağı: Tilt={current_tilt}° | {response}")
                elif command == 'a':  # Sol
                    current_pan = max(10, current_pan - step_size)
                    response = self.send_command(f"PAN,{current_pan}")
                    print(f"Sol: Pan={current_pan}° | {response}")
                elif command == 'd':  # Sağ
                    current_pan = min(170, current_pan + step_size)
                    response = self.send_command(f"PAN,{current_pan}")
                    print(f"Sağ: Pan={current_pan}° | {response}")
                elif command == 'c':  # Merkez
                    current_pan, current_tilt = 90, 90
                    response = self.send_command("CENTER")
                    print(f"Merkez: {response}")
                elif command == 'l':  # Lazer
                    laser_on = not laser_on
                    laser_cmd = "ON" if laser_on else "OFF"
                    response = self.send_command(f"LASER,{laser_cmd}")
                    print(f"Lazer {laser_cmd}: {response}")
                elif command == 't':  # Status
                    response = self.send_command("STATUS")
                    print(f"Durum: {response}")
                else:
                    print("Geçersiz komut!")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Hata: {e}")
        
        print("\\n👋 İnteraktif kontrol sonlandırıldı")
    
    def run_full_diagnosis(self):
        """Tam teşhis çalıştır"""
        print("🎯 BARLAS PAN-TILT TEŞHİS ARACI")
        print("=" * 60)
        print("Bu araç servo hareket problemlerini tespit eder")
        print("=" * 60)
        
        # 1. Arduino'yu bul
        print("\\n📍 ADIM 1: Arduino Port Tespiti")
        port = self.scan_all_ports()
        
        if not port:
            print("\\n❌ TEŞHIS SONUCU: Arduino bulunamadı!")
            print("\\n🔧 ÇÖZÜMLEr:")
            print("1. Arduino USB kablonu kontrol edin")
            print("2. Arduino'ya firmware yüklendiğinden emin olun")
            print("3. Başka program Arduino'yu kullanıyor olabilir")
            print("4. Arduino IDE Serial Monitor kapalı olmalı")
            return False
        
        # 2. Arduino'ya bağlan
        print("\\n📍 ADIM 2: Arduino Bağlantısı")
        if not self.connect_to_arduino(port):
            print("\\n❌ TEŞHIS SONUCU: Arduino bağlantısı başarısız!")
            return False
        
        # 3. Servo hareket testi
        print("\\n📍 ADIM 3: Servo Hareket Testi")
        servo_ok = self.test_servo_movement()
        
        # 4. Lazer testi
        print("\\n📍 ADIM 4: Lazer Sistemi Testi")
        laser_ok = self.test_laser()
        
        # 5. Sonuç raporu
        print("\\n" + "=" * 60)
        print("🎯 TEŞHİS RAPORU")
        print("=" * 60)
        print(f"Arduino Portu: {port}")
        print(f"Arduino Bağlantısı: ✅ Başarılı")
        print(f"Servo Hareketi: {'✅ Başarılı' if servo_ok else '❌ Başarısız'}")
        print(f"Lazer Sistemi: {'✅ Başarılı' if laser_ok else '❌ Başarısız'}")
        
        overall_status = servo_ok and laser_ok
        
        if overall_status:
            print("\\n🎉 GENEL SONUÇ: SİSTEM TAMAMEN ÇALIŞIYOR!")
            print("\\n✅ Pan-Tilt sistemi hazır, dart hedefleme yapılabilir")
            
            # İnteraktif kontrol teklifi
            choice = input("\\nİnteraktif kontrol yapmak ister misiniz? (y/n): ").strip().lower()
            if choice == 'y':
                self.interactive_control()
                
        else:
            print("\\n⚠️  GENEL SONUÇ: SİSTEMDE SORUN VAR!")
            print("\\n🔧 ÖNERİLER:")
            
            if not servo_ok:
                print("• Servo bağlantılarını kontrol edin")
                print("• Servo güç beslemesini kontrol edin (5V)")
                print("• Servo kablolarının doğru pin'lere bağlı olduğunu kontrol edin")
            
            if not laser_ok:
                print("• Lazer modül bağlantısını kontrol edin")
                print("• Lazer modül güç beslemesini kontrol edin")
        
        # Bağlantıyı kapat
        if self.serial_conn:
            self.serial_conn.close()
        
        return overall_status

def main():
    """Ana fonksiyon"""
    diagnosis = PanTiltDiagnosis()
    
    try:
        diagnosis.run_full_diagnosis()
    except KeyboardInterrupt:
        print("\\n\\n👋 Teşhis iptal edildi")
    except Exception as e:
        print(f"\\n❌ Beklenmeyen hata: {e}")
    finally:
        if diagnosis.serial_conn:
            diagnosis.serial_conn.close()

if __name__ == "__main__":
    main()
