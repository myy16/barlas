#!/usr/bin/env python3
"""
🔌 BARLAS PIN TEST VE DOĞRULAMA SİSTEMİ
Her pin bağlantısını tek tek test eder
"""

import sys
import os
import time
import serial

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

class BarlasPinTester:
    """🔌 Pin test sistemi"""
    
    def __init__(self, port="COM3"):
        self.port = port
        self.ser = None
        
        # Sabit pin atamaları (Arduino kodundan)
        self.pin_map = {
            # Pan-Tilt
            "pan_servo": 9,
            "tilt_servo": 10,
            "laser": 13,
            
            # Motor Sürücü
            "motor_r_en": 2,
            "motor_l_en": 3,
            "motor_rpwm": 4,
            "motor_lpwm": 5,
            "motor_r_is": "A0",
            "motor_l_is": "A1",
            
            # Fren & Far
            "brake_servo": 6,
            "headlight": 7,
            
            # Encoders (Interrupt)
            "encoder1_a": 21,
            "encoder1_b": 20,
            "encoder2_a": 19,
            "encoder2_b": 18
        }
        
        print("🔌 BARLAS PIN TEST SİSTEMİ")
        print("=" * 40)
    
    def connect(self):
        """Arduino'ya bağlan"""
        try:
            self.ser = serial.Serial(self.port, 9600, timeout=2)
            time.sleep(2)  # Arduino reset bekle
            print(f"✅ Arduino bağlantısı: {self.port}")
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def send_command(self, command):
        """Arduino'ya komut gönder"""
        if not self.ser:
            return "No connection"
        
        try:
            self.ser.write(f"{command}\n".encode())
            time.sleep(0.1)
            response = self.ser.readline().decode().strip()
            return response
        except Exception as e:
            return f"Error: {e}"
    
    def test_basic_communication(self):
        """Temel iletişim testi"""
        print("\n📡 TEMEL İLETİŞİM TESTİ")
        print("-" * 30)
        
        response = self.send_command("TEST")
        if "OK" in response:
            print(f"✅ İletişim: {response}")
            return True
        else:
            print(f"❌ İletişim hatası: {response}")
            return False
    
    def test_laser_pin(self):
        """Lazer pin testi (Pin 13)"""
        print(f"\n🔥 LAZER PİN TESTİ (Pin {self.pin_map['laser']})")
        print("-" * 35)
        
        # Lazer aç
        response1 = self.send_command("LASER,ON")
        print(f"  Lazer ON: {response1}")
        time.sleep(1)
        
        # Lazer kapat
        response2 = self.send_command("LASER,OFF") 
        print(f"  Lazer OFF: {response2}")
        
        success = "Laser ON" in response1 and "Laser OFF" in response2
        print(f"  {'✅' if success else '❌'} Lazer Pin: {'OK' if success else 'HATA'}")
        return success
    
    def test_servo_pins(self):
        """Servo pin testleri (Pin 9, 10, 6)"""
        print(f"\n🎯 SERVO PİN TESTİ")
        print("-" * 25)
        
        # Pan-Tilt test
        response1 = self.send_command("MOVE,45,45")
        print(f"  Pan-Tilt (Pin {self.pin_map['pan_servo']},{self.pin_map['tilt_servo']}): {response1}")
        time.sleep(1)
        
        response2 = self.send_command("MOVE,90,90")
        print(f"  Merkez pozisyon: {response2}")
        time.sleep(1)
        
        # Fren servo test
        response3 = self.send_command("BRAKE_ON")
        print(f"  Fren Servo (Pin {self.pin_map['brake_servo']}): {response3}")
        time.sleep(0.5)
        
        response4 = self.send_command("BRAKE_OFF")
        print(f"  Fren Off: {response4}")
        
        success = all("Moved" in r or "Brake" in r for r in [response1, response2, response3, response4])
        print(f"  {'✅' if success else '❌'} Servo Pinleri: {'OK' if success else 'HATA'}")
        return success
    
    def test_motor_pins(self):
        """Motor pin testleri (Pin 2,3,4,5)"""
        print(f"\n🚗 MOTOR PİN TESTİ")
        print("-" * 25)
        print(f"  Enable Pinleri: {self.pin_map['motor_r_en']}, {self.pin_map['motor_l_en']}")
        print(f"  PWM Pinleri: {self.pin_map['motor_rpwm']}, {self.pin_map['motor_lpwm']}")
        
        # Düşük hızda test
        response1 = self.send_command("MOTOR_FORWARD,50")
        print(f"  Motor İleri: {response1}")
        time.sleep(1)
        
        response2 = self.send_command("MOTOR_STOP")
        print(f"  Motor Stop: {response2}")
        time.sleep(0.5)
        
        response3 = self.send_command("MOTOR_BACKWARD,50")
        print(f"  Motor Geri: {response3}")
        time.sleep(1)
        
        response4 = self.send_command("MOTOR_STOP")
        print(f"  Motor Stop: {response4}")
        
        success = all("Motor" in r for r in [response1, response2, response3, response4])
        print(f"  {'✅' if success else '❌'} Motor Pinleri: {'OK' if success else 'HATA'}")
        return success
    
    def test_headlight_pin(self):
        """Far pin testi (Pin 7)"""
        print(f"\n💡 FAR PİN TESTİ (Pin {self.pin_map['headlight']})")
        print("-" * 30)
        
        response1 = self.send_command("HEADLIGHT_ON")
        print(f"  Far ON: {response1}")
        time.sleep(1)
        
        response2 = self.send_command("HEADLIGHT_OFF")
        print(f"  Far OFF: {response2}")
        
        success = "Headlights ON" in response1 and "Headlights OFF" in response2
        print(f"  {'✅' if success else '❌'} Far Pin: {'OK' if success else 'HATA'}")
        return success
    
    def test_encoder_pins(self):
        """Encoder pin testleri (Pin 18,19,20,21)"""
        print(f"\n📊 ENCODER PİN TESTİ (Interrupt Pinleri)")
        print("-" * 45)
        print(f"  Sol Encoder: Pin {self.pin_map['encoder1_a']},{self.pin_map['encoder1_b']}")
        print(f"  Sağ Encoder: Pin {self.pin_map['encoder2_a']},{self.pin_map['encoder2_b']}")
        
        # Encoder sıfırla
        response1 = self.send_command("RESET_ENCODERS")
        print(f"  Reset: {response1}")
        time.sleep(0.5)
        
        # İlk okuma
        response2 = self.send_command("GET_ENCODERS")
        print(f"  İlk okuma: {response2}")
        
        # Motor çalıştır (encoder değişmeli)
        print("  Motor çalıştırılıyor (encoder değişimi için)...")
        self.send_command("MOTOR_FORWARD,80")
        time.sleep(2)
        self.send_command("MOTOR_STOP")
        
        # İkinci okuma
        response3 = self.send_command("GET_ENCODERS")
        print(f"  İkinci okuma: {response3}")
        
        # Encoder değişti mi kontrol et
        if "ENCODER1:" in response3 and "ENCODER2:" in response3:
            # Encoder değerlerini parse et
            try:
                parts = response3.split(",")
                enc1 = int(parts[0].split(":")[1])
                enc2 = int(parts[1].split(":")[1])
                
                if abs(enc1) > 0 or abs(enc2) > 0:
                    print(f"  ✅ Encoder değişimi tespit edildi: Sol={enc1}, Sağ={enc2}")
                    return True
                else:
                    print(f"  ⚠️  Encoder değişimi yok (bağlantı problemi olabilir)")
                    return False
            except:
                print(f"  ❌ Encoder verisi parse edilemedi")
                return False
        else:
            print(f"  ❌ Encoder cevabı alınamadı")
            return False
    
    def test_system_status(self):
        """Sistem durumu testi"""
        print(f"\n📋 SİSTEM DURUM TESTİ")
        print("-" * 30)
        
        response = self.send_command("STATUS")
        print(f"Sistem durumu:\n{response}")
        
        return "BARLAS" in response
    
    def run_comprehensive_test(self):
        """Kapsamlı pin testi"""
        if not self.connect():
            return False
        
        print("\n🧪 KAPSAMLI PİN TESTİ BAŞLIYOR...")
        print("=" * 50)
        
        test_results = []
        
        # Tüm testleri çalıştır
        test_results.append(("İletişim", self.test_basic_communication()))
        test_results.append(("Lazer Pin", self.test_laser_pin()))
        test_results.append(("Servo Pinleri", self.test_servo_pins()))
        test_results.append(("Motor Pinleri", self.test_motor_pins()))
        test_results.append(("Far Pin", self.test_headlight_pin()))
        test_results.append(("Encoder Pinleri", self.test_encoder_pins()))
        test_results.append(("Sistem Durumu", self.test_system_status()))
        
        # Sonuçları özetle
        print("\n" + "=" * 50)
        print("📋 TEST SONUÇLARI")
        print("=" * 50)
        
        passed = 0
        total = len(test_results)
        
        for test_name, result in test_results:
            status = "✅ GEÇTİ" if result else "❌ BAŞARISIZ"
            print(f"  {test_name:15} : {status}")
            if result:
                passed += 1
        
        print("\n" + "=" * 50)
        print(f"ÖZET: {passed}/{total} test başarılı")
        
        if passed == total:
            print("🎉 TÜM PİN BAĞLANTILARI DOĞRU!")
        else:
            print("⚠️  BAZI PIN BAĞLANTILARI HATALI!")
            print("\nÖNERİLER:")
            print("- Bağlantıları kontrol edin")
            print("- PIN_KONFIGÜRASYONU.md dosyasını inceleyin")
            print("- Multimetre ile voltaj ölçümü yapın")
        
        return passed == total
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.ser:
            self.ser.close()
            print("🔌 Arduino bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    print("🔌 BARLAS PIN TEST VE DOĞRULAMA SİSTEMİ")
    print("⚠️  Bu test fiziksel bağlantıları doğrular!")
    print()
    
    # Port seçimi
    port = input("Arduino portu (varsayılan COM3): ") or "COM3"
    
    # Test sistemini başlat
    tester = BarlasPinTester(port=port)
    
    try:
        success = tester.run_comprehensive_test()
        
        if success:
            print("\n✅ Sistem hazır! Ana programı çalıştırabilirsiniz.")
        else:
            print("\n❌ Pin bağlantı sorunları var!")
            
    finally:
        tester.disconnect()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Test durduruldu!")
    except Exception as e:
        print(f"\n💥 Test hatası: {e}")
