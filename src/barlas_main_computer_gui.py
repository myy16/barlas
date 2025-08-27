#!/usr/bin/env python3
"""
BARLAS Ana Bilgisayar Kontrol Interface
MSI Laptop i5-13420H + RTX 4060 İçin
Hibrit Sistem Yönetim ve Kontrol Paneli
"""

import tkinter as tk
from tkinter import ttk, messagebox
import rospy
import threading
import subprocess
import json
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class BARLASControlInterface:
    """BARLAS Ana Bilgisayar GUI Kontrol Paneli"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("BARLAS Ana Bilgisayar Kontrol Merkezi")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2C3E50')
        
        # ROS initialization
        try:
            rospy.init_node('barlas_control_gui', anonymous=True)
            self.ros_active = True
        except:
            self.ros_active = False
            messagebox.showwarning("ROS Uyarısı", "ROS Master bağlantısı yok!\nBazı özellikler çalışmayabilir.")
        
        # System state
        self.system_status = {
            'ros_connected': False,
            'mavros_connected': False,
            'dart_targeting': False,
            'obstacle_avoidance': True,
            'mission_active': False
        }
        
        self.sensor_data = {
            'ultrasonic': [0.0] * 8,
            'battery_voltage': 0.0,
            'temperature': 0.0
        }
        
        self.setup_gui()
        self.setup_ros_connections()
        
        # Update timer
        self.update_timer()
    
    def setup_gui(self):
        """GUI bileşenlerini oluştur"""
        
        # Ana container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # === ÜST PANEL: Sistem Durumu ===
        status_frame = ttk.LabelFrame(main_frame, text="🎯 BARLAS Sistem Durumu", padding=10)
        status_frame.pack(fill='x', pady=(0, 10))
        
        # Status indicators
        self.status_labels = {}
        status_items = [
            ('ROS Master', 'ros_connected'),
            ('MAVROS', 'mavros_connected'), 
            ('Dart Targeting', 'dart_targeting'),
            ('Engel Kaçınma', 'obstacle_avoidance'),
            ('Görev Aktif', 'mission_active')
        ]
        
        for i, (text, key) in enumerate(status_items):
            label = tk.Label(status_frame, text=f"{text}: ❌", font=('Arial', 10, 'bold'))
            label.grid(row=0, column=i, padx=10)
            self.status_labels[key] = label
        
        # === ORTA SOL: Kontrol Butonları ===
        control_frame = ttk.LabelFrame(main_frame, text="🎮 Sistem Kontrol", padding=10)
        control_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # Mission Control
        mission_frame = ttk.LabelFrame(control_frame, text="Görev Kontrol", padding=5)
        mission_frame.pack(fill='x', pady=5)
        
        self.start_mission_btn = tk.Button(mission_frame, text="🚀 Görevi Başlat", 
                                          command=self.start_mission, bg='#27AE60', fg='white', font=('Arial', 10, 'bold'))
        self.start_mission_btn.pack(side='left', padx=5)
        
        self.stop_mission_btn = tk.Button(mission_frame, text="🛑 Görevi Durdur", 
                                         command=self.stop_mission, bg='#E74C3C', fg='white', font=('Arial', 10, 'bold'))
        self.stop_mission_btn.pack(side='left', padx=5)
        
        self.emergency_btn = tk.Button(mission_frame, text="⚠️ ACİL DURDUR", 
                                      command=self.emergency_stop, bg='#8B0000', fg='white', font=('Arial', 12, 'bold'))
        self.emergency_btn.pack(side='right', padx=5)
        
        # Mode Selection
        mode_frame = ttk.LabelFrame(control_frame, text="Sistem Modu", padding=5)
        mode_frame.pack(fill='x', pady=5)
        
        self.mode_var = tk.StringVar(value="AUTONOMOUS")
        modes = [("🚗 Otonom", "AUTONOMOUS"), ("🎯 Dart Targeting", "DART_TARGETING"), ("🎮 Manuel", "MANUAL")]
        
        for text, value in modes:
            tk.Radiobutton(mode_frame, text=text, variable=self.mode_var, value=value,
                          command=self.change_mode, font=('Arial', 9)).pack(anchor='w')
        
        # Dart Control
        dart_frame = ttk.LabelFrame(control_frame, text="🎯 Dart Hedefleme", padding=5)
        dart_frame.pack(fill='x', pady=5)
        
        self.dart_enable_btn = tk.Button(dart_frame, text="Dart Targeting AKTİF", 
                                        command=self.toggle_dart_targeting, bg='#F39C12', fg='white')
        self.dart_enable_btn.pack(fill='x', pady=2)
        
        self.laser_fire_btn = tk.Button(dart_frame, text="🔥 LAZER ATEŞ", 
                                       command=self.fire_laser, bg='#C0392B', fg='white')
        self.laser_fire_btn.pack(fill='x', pady=2)
        
        # Manual Control
        manual_frame = ttk.LabelFrame(control_frame, text="🎮 Manuel Kontrol", padding=5)
        manual_frame.pack(fill='x', pady=5)
        
        # Speed control
        tk.Label(manual_frame, text="Hız:").pack()
        self.speed_scale = tk.Scale(manual_frame, from_=-1.0, to=1.0, resolution=0.1, 
                                   orient='horizontal', command=self.manual_control)
        self.speed_scale.pack(fill='x')
        
        tk.Label(manual_frame, text="Dönüş:").pack()
        self.turn_scale = tk.Scale(manual_frame, from_=-1.0, to=1.0, resolution=0.1, 
                                  orient='horizontal', command=self.manual_control)
        self.turn_scale.pack(fill='x')
        
        # === ORTA SAĞ: Sensör Verileri ===
        sensor_frame = ttk.LabelFrame(main_frame, text="📡 Sensör Verileri", padding=10)
        sensor_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))
        
        # Ultrasonik radar görünümü
        self.setup_radar_plot(sensor_frame)
        
        # Sensör değerleri
        values_frame = ttk.Frame(sensor_frame)
        values_frame.pack(fill='x', pady=10)
        
        self.battery_label = tk.Label(values_frame, text="🔋 Batarya: 0.0V", font=('Arial', 10))
        self.battery_label.pack(anchor='w')
        
        self.temp_label = tk.Label(values_frame, text="🌡️ Sıcaklık: 0.0°C", font=('Arial', 10))
        self.temp_label.pack(anchor='w')
        
        # === ALT PANEL: Konsol ===
        console_frame = ttk.LabelFrame(main_frame, text="📋 Sistem Konsolu", padding=5)
        console_frame.pack(fill='x', pady=(10, 0))
        
        self.console_text = tk.Text(console_frame, height=8, bg='black', fg='green', font=('Courier', 9))
        console_scroll = ttk.Scrollbar(console_frame, orient='vertical', command=self.console_text.yview)
        self.console_text.configure(yscrollcommand=console_scroll.set)
        
        self.console_text.pack(side='left', fill='both', expand=True)
        console_scroll.pack(side='right', fill='y')
        
        self.log_message("🎯 BARLAS Kontrol Interface başlatıldı")
    
    def setup_radar_plot(self, parent):
        """Ultrasonik sensörler için radar görünümü oluştur"""
        self.fig, self.ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(projection='polar'))
        self.ax.set_ylim(0, 5)
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_title("Ultrasonik Sensör Radar")
        
        # 8 sensör pozisyonu (radyan)
        self.sensor_angles = np.array([0, np.pi/4, np.pi/2, 3*np.pi/4, 
                                      np.pi, 5*np.pi/4, 3*np.pi/2, 7*np.pi/4])
        
        self.radar_canvas = FigureCanvasTkAgg(self.fig, parent)
        self.radar_canvas.get_tk_widget().pack(fill='both', expand=True)
    
    def setup_ros_connections(self):
        """ROS bağlantılarını kur"""
        if not self.ros_active:
            return
        
        try:
            # Subscribers
            rospy.Subscriber('/barlas/system/status', String, self.system_status_callback)
            rospy.Subscriber('/barlas/sensors/ultrasonic', LaserScan, self.ultrasonic_callback)
            rospy.Subscriber('/barlas/sensors/all_data', String, self.sensor_data_callback)
            
            # Publishers
            self.cmd_vel_pub = rospy.Publisher('/barlas/navigation/cmd_vel', Twist, queue_size=1)
            self.dart_enable_pub = rospy.Publisher('/barlas/dart_laser/targeting_enable', Bool, queue_size=1)
            self.laser_fire_pub = rospy.Publisher('/barlas/servo/laser_command', Bool, queue_size=1)
            
            self.log_message("✅ ROS bağlantıları kuruldu")
        except Exception as e:
            self.log_message(f"❌ ROS bağlantı hatası: {e}")
    
    def system_status_callback(self, msg):
        """Sistem durumu güncellemesi"""
        try:
            status = json.loads(msg.data)
            self.system_status.update(status)
            self.update_status_display()
        except:
            pass
    
    def ultrasonic_callback(self, msg):
        """Ultrasonik sensör verilerini güncelle"""
        if len(msg.ranges) >= 8:
            self.sensor_data['ultrasonic'] = msg.ranges[:8]
            self.update_radar_plot()
    
    def sensor_data_callback(self, msg):
        """Genel sensör verilerini güncelle"""
        try:
            data = json.loads(msg.data)
            if 'battery_voltage' in data:
                self.sensor_data['battery_voltage'] = data['battery_voltage']
            if 'temperature' in data:
                self.sensor_data['temperature'] = data['temperature']
            self.update_sensor_display()
        except:
            pass
    
    def update_status_display(self):
        """Durum göstergelerini güncelle"""
        for key, label in self.status_labels.items():
            if key in self.system_status:
                status = "✅" if self.system_status[key] else "❌"
                text = key.replace('_', ' ').title()
                label.config(text=f"{text}: {status}")
    
    def update_radar_plot(self):
        """Radar plotunu güncelle"""
        self.ax.clear()
        self.ax.set_ylim(0, 5)
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_title("Ultrasonik Sensör Radar", pad=20)
        
        distances = self.sensor_data['ultrasonic']
        colors = ['red' if d < 1.0 else 'yellow' if d < 2.0 else 'green' for d in distances]
        
        self.ax.scatter(self.sensor_angles, distances, c=colors, s=100, alpha=0.7)
        
        # Danger zones
        self.ax.fill_between(np.linspace(0, 2*np.pi, 100), 0, 1, alpha=0.2, color='red', label='Tehlike')
        self.ax.fill_between(np.linspace(0, 2*np.pi, 100), 1, 2, alpha=0.1, color='yellow', label='Uyarı')
        
        self.radar_canvas.draw()
    
    def update_sensor_display(self):
        """Sensör değer displayini güncelle"""
        battery = self.sensor_data['battery_voltage']
        temp = self.sensor_data['temperature']
        
        # Renk kodlaması
        bat_color = 'green' if battery > 11.0 else 'orange' if battery > 10.0 else 'red'
        temp_color = 'green' if temp < 60 else 'orange' if temp < 80 else 'red'
        
        self.battery_label.config(text=f"🔋 Batarya: {battery:.1f}V", fg=bat_color)
        self.temp_label.config(text=f"🌡️ Sıcaklık: {temp:.1f}°C", fg=temp_color)
    
    # Control Methods
    def start_mission(self):
        """Görevi başlat"""
        self.log_message("🚀 Görev başlatılıyor...")
        if self.ros_active:
            # ROS service call veya topic publish
            pass
    
    def stop_mission(self):
        """Görevi durdur"""
        self.log_message("🛑 Görev durduruluyor...")
        if self.ros_active:
            # Stop mission
            pass
    
    def emergency_stop(self):
        """Acil durdurma"""
        result = messagebox.askyesno("ACİL DURDURMA", "Sistemi acil durdurmak istediğinizden emin misiniz?")
        if result:
            self.log_message("⚠️ ACİL DURDURMA AKTİF!")
            if self.ros_active:
                # Emergency stop procedures
                pass
    
    def change_mode(self):
        """Sistem modunu değiştir"""
        mode = self.mode_var.get()
        self.log_message(f"🔄 Sistem modu değiştirildi: {mode}")
    
    def toggle_dart_targeting(self):
        """Dart hedeflemeyi aç/kapat"""
        if self.ros_active:
            current = self.system_status.get('dart_targeting', False)
            new_state = not current
            
            msg = Bool()
            msg.data = new_state
            self.dart_enable_pub.publish(msg)
            
            status = "AKTİF" if new_state else "PASİF"
            self.log_message(f"🎯 Dart targeting {status}")
    
    def fire_laser(self):
        """Lazer ateşle"""
        if self.ros_active:
            msg = Bool()
            msg.data = True
            self.laser_fire_pub.publish(msg)
            self.log_message("🔥 LAZER ATEŞ!")
            
            # 1 saniye sonra kapat
            self.root.after(1000, self.turn_off_laser)
    
    def turn_off_laser(self):
        """Lazeri kapat"""
        if self.ros_active:
            msg = Bool()
            msg.data = False
            self.laser_fire_pub.publish(msg)
            self.log_message("🔴 Lazer kapatıldı")
    
    def manual_control(self, value=None):
        """Manuel araç kontrolü"""
        if self.ros_active and self.mode_var.get() == "MANUAL":
            linear = self.speed_scale.get()
            angular = self.turn_scale.get()
            
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.cmd_vel_pub.publish(cmd)
    
    def log_message(self, message):
        """Konsola mesaj yazdır"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_line = f"[{timestamp}] {message}\n"
        
        self.console_text.insert(tk.END, log_line)
        self.console_text.see(tk.END)
        
        # Son 100 satırı tut
        lines = self.console_text.get("1.0", tk.END).split('\n')
        if len(lines) > 100:
            self.console_text.delete("1.0", f"{len(lines)-100}.0")
    
    def update_timer(self):
        """Periyodik güncelleme"""
        if self.ros_active:
            # ROS durumunu kontrol et
            try:
                rospy.get_published_topics()
                self.system_status['ros_connected'] = True
            except:
                self.system_status['ros_connected'] = False
        
        self.update_status_display()
        
        # 1 saniye sonra tekrar çağır
        self.root.after(1000, self.update_timer)
    
    def run(self):
        """GUI'yi çalıştır"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.log_message("🛑 Sistem kapatılıyor...")
            if self.ros_active:
                rospy.signal_shutdown("GUI closed")

def main():
    """Ana fonksiyon"""
    print("🎯 BARLAS Ana Bilgisayar Kontrol Interface")
    print("MSI Laptop i5-13420H + RTX 4060")
    print("==========================================")
    
    try:
        app = BARLASControlInterface()
        app.run()
    except Exception as e:
        print(f"❌ Hata: {e}")

if __name__ == '__main__':
    main()
