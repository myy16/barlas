#!/usr/bin/env python3
"""
BARLAS GPS Sensor Module
Neo-M8N GPS modülü ile konum takibi
Teknofest İnsansız Kara Aracı Yarışması
"""

import rospy
import serial
import time
import threading
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class GPSSensor:
    def __init__(self):
        """GPS sensor initialize"""
        rospy.init_node('barlas_gps_sensor', anonymous=True)
        
        # Parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB2')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.frame_id = rospy.get_param('~frame_id', 'gps_frame')
        self.simulation_mode = rospy.get_param('~simulation_mode', False)
        
        # Publishers
        self.gps_fix_pub = rospy.Publisher('/barlas/sensors/gps', NavSatFix, queue_size=1)
        self.gps_vel_pub = rospy.Publisher('/barlas/sensors/gps_velocity', TwistStamped, queue_size=1)
        
        # GPS data
        self.current_fix = NavSatFix()
        self.current_velocity = TwistStamped()
        self.lock = threading.Lock()
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        
        # Initialize
        self.setup_gps()
        
        rospy.loginfo("🛰️ BARLAS GPS Sensor başlatıldı")
    
    def setup_gps(self):
        """GPS bağlantısını kur"""
        if self.simulation_mode:
            rospy.loginfo("📡 GPS simülasyon modu aktif")
            self.start_simulation()
            return
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            rospy.loginfo(f"📡 GPS bağlantısı kuruldu: {self.port}")
            self.running = True
            self.start_reading()
            
        except Exception as e:
            rospy.logerr(f"❌ GPS bağlantı hatası: {e}")
            self.start_simulation()  # Fallback to simulation
    
    def start_reading(self):
        """GPS veri okuma thread'ini başlat"""
        self.read_thread = threading.Thread(target=self.read_gps_data)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        # Publisher thread
        self.pub_thread = threading.Thread(target=self.publish_data)
        self.pub_thread.daemon = True
        self.pub_thread.start()
    
    def read_gps_data(self):
        """GPS verilerini oku (NMEA format)"""
        while self.running and not rospy.is_shutdown():
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('ascii', errors='replace').strip()
                    self.parse_nmea_sentence(line)
                    
                time.sleep(0.1)  # 10Hz okuma
                
            except Exception as e:
                rospy.logwarn(f"GPS okuma hatası: {e}")
                time.sleep(1)
    
    def parse_nmea_sentence(self, sentence):
        """NMEA cümlesini parse et"""
        try:
            if not sentence.startswith('$'):
                return
                
            parts = sentence.split(',')
            sentence_type = parts[0]
            
            # $GPGGA - Global Positioning System Fix Data
            if sentence_type == '$GPGGA':
                self.parse_gga(parts)
            
            # $GPRMC - Recommended Minimum Course
            elif sentence_type == '$GPRMC':
                self.parse_rmc(parts)
                
        except Exception as e:
            rospy.logwarn(f"NMEA parse hatası: {e}")
    
    def parse_gga(self, parts):
        """GGA sentence parse (position data)"""
        try:
            if len(parts) < 15:
                return
            
            # GPS fix quality
            fix_quality = int(parts[6]) if parts[6] else 0
            
            if fix_quality == 0:
                return  # No fix
            
            with self.lock:
                # Time
                self.current_fix.header = Header()
                self.current_fix.header.stamp = rospy.Time.now()
                self.current_fix.header.frame_id = self.frame_id
                
                # Latitude
                if parts[2] and parts[3]:
                    lat_deg = float(parts[2][:2])
                    lat_min = float(parts[2][2:])
                    latitude = lat_deg + lat_min / 60.0
                    if parts[3] == 'S':
                        latitude = -latitude
                    self.current_fix.latitude = latitude
                
                # Longitude  
                if parts[4] and parts[5]:
                    lon_deg = float(parts[4][:3])
                    lon_min = float(parts[4][3:])
                    longitude = lon_deg + lon_min / 60.0
                    if parts[5] == 'W':
                        longitude = -longitude
                    self.current_fix.longitude = longitude
                
                # Altitude
                if parts[9]:
                    self.current_fix.altitude = float(parts[9])
                
                # Fix status
                if fix_quality >= 1:
                    self.current_fix.status.status = NavSatFix.STATUS_FIX
                else:
                    self.current_fix.status.status = NavSatFix.STATUS_NO_FIX
                    
        except Exception as e:
            rospy.logwarn(f"GGA parse hatası: {e}")
    
    def parse_rmc(self, parts):
        """RMC sentence parse (velocity data)"""
        try:
            if len(parts) < 12:
                return
            
            # Speed over ground (knots)
            if parts[7]:
                speed_knots = float(parts[7])
                speed_ms = speed_knots * 0.514444  # knots to m/s
                
                # Course over ground
                course = float(parts[8]) if parts[8] else 0.0
                course_rad = course * 3.14159 / 180.0
                
                with self.lock:
                    self.current_velocity.header = Header()
                    self.current_velocity.header.stamp = rospy.Time.now()
                    self.current_velocity.header.frame_id = self.frame_id
                    
                    # Linear velocity
                    self.current_velocity.twist.linear.x = speed_ms
                    self.current_velocity.twist.angular.z = course_rad
                    
        except Exception as e:
            rospy.logwarn(f"RMC parse hatası: {e}")
    
    def start_simulation(self):
        """GPS simülasyonu başlat"""
        rospy.loginfo("🎮 GPS simülasyon modu başlatılıyor...")
        
        self.running = True
        sim_thread = threading.Thread(target=self.simulate_gps_data)
        sim_thread.daemon = True
        sim_thread.start()
        
        # Publisher thread
        self.pub_thread = threading.Thread(target=self.publish_data)
        self.pub_thread.daemon = True
        self.pub_thread.start()
    
    def simulate_gps_data(self):
        """GPS simülasyon verisi üret"""
        # Teknofest yarışma alanı koordinatları (örnek)
        base_lat = 41.015137  # İstanbul
        base_lon = 28.976442
        base_alt = 50.0
        
        time_offset = 0.0
        
        while self.running and not rospy.is_shutdown():
            try:
                with self.lock:
                    # Header
                    self.current_fix.header = Header()
                    self.current_fix.header.stamp = rospy.Time.now()
                    self.current_fix.header.frame_id = self.frame_id
                    
                    # Simulated position (circular movement)
                    radius = 0.0001  # ~11 meter
                    time_offset += 0.1
                    
                    self.current_fix.latitude = base_lat + radius * (time_offset * 0.1) % 0.0002
                    self.current_fix.longitude = base_lon + radius * (time_offset * 0.1) % 0.0002
                    self.current_fix.altitude = base_alt + 2.0 * (time_offset % 10.0)
                    
                    # Fix status
                    self.current_fix.status.status = NavSatFix.STATUS_FIX
                    self.current_fix.status.service = NavSatFix.SERVICE_GPS
                    
                    # Simulated velocity
                    self.current_velocity.header = self.current_fix.header
                    self.current_velocity.twist.linear.x = 1.0 + 0.5 * (time_offset % 5.0)
                    self.current_velocity.twist.angular.z = 0.1 * (time_offset % 3.0)
                
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                rospy.logwarn(f"GPS simülasyon hatası: {e}")
                time.sleep(1)
    
    def publish_data(self):
        """GPS verilerini publish et"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            try:
                with self.lock:
                    if self.current_fix.header.stamp.secs > 0:
                        self.gps_fix_pub.publish(self.current_fix)
                        
                    if self.current_velocity.header.stamp.secs > 0:
                        self.gps_vel_pub.publish(self.current_velocity)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logwarn(f"GPS publish hatası: {e}")
                time.sleep(1)
    
    def get_current_position(self):
        """Mevcut GPS konumunu döndür"""
        with self.lock:
            return {
                'latitude': self.current_fix.latitude,
                'longitude': self.current_fix.longitude,
                'altitude': self.current_fix.altitude,
                'fix_status': self.current_fix.status.status
            }
    
    def get_current_velocity(self):
        """Mevcut GPS hızını döndür"""
        with self.lock:
            return {
                'linear_velocity': self.current_velocity.twist.linear.x,
                'course': self.current_velocity.twist.angular.z
            }
    
    def cleanup(self):
        """Cleanup GPS connection"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()

def main():
    """Ana fonksiyon"""
    try:
        gps_sensor = GPSSensor()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("GPS sensor kapatılıyor...")
    except Exception as e:
        rospy.logerr(f"GPS sensor hatası: {e}")
    finally:
        if 'gps_sensor' in locals():
            gps_sensor.cleanup()

if __name__ == '__main__':
    main()
