#!/usr/bin/env python3
"""
TEKNOFEST 2025 İNSANSIZ KARA ARACI YARIŞMASI
Test ve Debug Utilities - Son Güncellenme: 27.12.2024

Bu modül yarışma sistemi test ve debug araçlarını içerir.
Sistem performansını analiz etmek ve sorunları tespit etmek için kullanılır.
"""

import os
import sys
import time
import json
import logging
import threading
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Any, Tuple
import numpy as np

try:
    import rospy
    from std_msgs.msg import String, Float32, Bool
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan, Image, Imu
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS not available - running in simulation mode")

@dataclass
class SystemMetrics:
    """Sistem performans metrikleri"""
    timestamp: float
    cpu_usage: float
    memory_usage: float
    disk_usage: float
    network_latency: float
    sensor_health: Dict[str, bool]
    competition_status: str
    current_task: str
    score: int

@dataclass
class TestResult:
    """Test sonucu veri yapısı"""
    test_name: str
    status: str  # "PASS", "FAIL", "WARNING"
    duration: float
    details: str
    timestamp: float

class BARLASSystemDebugger:
    """BARLAS sistem debug ve test araçları"""
    
    def __init__(self):
        self.test_results = []
        self.metrics_history = []
        self.is_monitoring = False
        self.logger = self._setup_logger()
        
        if ROS_AVAILABLE:
            self._init_ros()
        else:
            self.logger.warning("ROS not available - debugging in offline mode")
            
    def _setup_logger(self) -> logging.Logger:
        """Logger konfigürasyonu"""
        logger = logging.getLogger("barlas_debugger")
        logger.setLevel(logging.DEBUG)
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        
        # File handler
        log_dir = "/tmp/barlas_debug"
        os.makedirs(log_dir, exist_ok=True)
        file_handler = logging.FileHandler(f"{log_dir}/debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        file_handler.setLevel(logging.DEBUG)
        
        # Formatter
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        file_handler.setFormatter(formatter)
        
        logger.addHandler(console_handler)
        logger.addHandler(file_handler)
        
        return logger
        
    def _init_ros(self):
        """ROS node ve subscriber'ları başlat"""
        try:
            rospy.init_node('barlas_system_debugger', anonymous=True)
            
            # Subscribers
            rospy.Subscriber('/barlas/competition/status', String, self._competition_status_callback)
            rospy.Subscriber('/barlas/sensors/scan', LaserScan, self._lidar_callback)
            rospy.Subscriber('/barlas/sensors/imu', Imu, self._imu_callback)
            rospy.Subscriber('/barlas/control/cmd_vel', Twist, self._control_callback)
            
            self.logger.info("ROS debugging interface initialized")
            
        except Exception as e:
            self.logger.error(f"ROS initialization failed: {e}")
            
    def _competition_status_callback(self, msg):
        """Competition status callback"""
        self.logger.debug(f"Competition status: {msg.data}")
        
    def _lidar_callback(self, msg):
        """LiDAR data callback"""
        # LiDAR sağlık kontrolü
        if len(msg.ranges) == 0:
            self.logger.warning("LiDAR: No range data received")
        elif all(r == float('inf') for r in msg.ranges):
            self.logger.warning("LiDAR: All readings infinite")
            
    def _imu_callback(self, msg):
        """IMU data callback"""
        # IMU sağlık kontrolü
        accel_mag = np.sqrt(msg.linear_acceleration.x**2 + 
                           msg.linear_acceleration.y**2 + 
                           msg.linear_acceleration.z**2)
        if accel_mag < 8.0 or accel_mag > 12.0:
            self.logger.warning(f"IMU: Unusual acceleration magnitude: {accel_mag}")
            
    def _control_callback(self, msg):
        """Control command callback"""
        self.logger.debug(f"Control: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")

    def run_system_diagnostics(self) -> List[TestResult]:
        """Kapsamlı sistem tanı testleri"""
        self.logger.info("Starting comprehensive system diagnostics...")
        self.test_results.clear()
        
        # Test listesi
        tests = [
            self._test_ros_connectivity,
            self._test_sensor_health,
            self._test_competition_modules,
            self._test_navigation_stack,
            self._test_control_systems,
            self._test_safety_systems,
            self._test_communication,
            self._test_file_system,
            self._test_dependencies
        ]
        
        # Testleri çalıştır
        for test_func in tests:
            try:
                start_time = time.time()
                result = test_func()
                duration = time.time() - start_time
                
                if result is not None:
                    result.duration = duration
                    result.timestamp = time.time()
                    self.test_results.append(result)
                    
            except Exception as e:
                self.logger.error(f"Test {test_func.__name__} failed with exception: {e}")
                self.test_results.append(TestResult(
                    test_name=test_func.__name__,
                    status="FAIL",
                    duration=0.0,
                    details=f"Exception: {str(e)}",
                    timestamp=time.time()
                ))
                
        self._generate_diagnostic_report()
        return self.test_results
        
    def _test_ros_connectivity(self) -> TestResult:
        """ROS bağlantı testi"""
        if not ROS_AVAILABLE:
            return TestResult("ROS Connectivity", "FAIL", 0.0, "ROS not available")
            
        try:
            # Master connectivity
            import rosgraph
            master = rosgraph.Master('/test_node')
            master.getPid()
            
            # Core topics check
            topics = rospy.get_published_topics()
            core_topics = ['/rosout', '/rosout_agg']
            missing_topics = [t for t in core_topics if not any(t in topic[0] for topic in topics)]
            
            if missing_topics:
                return TestResult("ROS Connectivity", "WARNING", 0.0, 
                                f"Missing core topics: {missing_topics}")
            else:
                return TestResult("ROS Connectivity", "PASS", 0.0, "ROS master and core topics available")
                
        except Exception as e:
            return TestResult("ROS Connectivity", "FAIL", 0.0, f"ROS master not accessible: {e}")
            
    def _test_sensor_health(self) -> TestResult:
        """Sensör sağlık testi"""
        sensor_status = {
            "lidar": False,
            "imu": False,  
            "ultrasonic": False,
            "camera": False,
            "gps": False
        }
        
        issues = []
        
        if ROS_AVAILABLE:
            try:
                # Topic listesini al
                topics = dict(rospy.get_published_topics())
                
                # Sensör topic'lerini kontrol et
                if '/barlas/sensors/scan' in topics:
                    sensor_status["lidar"] = True
                else:
                    issues.append("LiDAR topic not found")
                    
                if '/barlas/sensors/imu' in topics:
                    sensor_status["imu"] = True
                else:
                    issues.append("IMU topic not found")
                    
                if '/barlas/sensors/ultrasonic' in topics:
                    sensor_status["ultrasonic"] = True
                else:
                    issues.append("Ultrasonic topic not found")
                    
                if '/barlas/camera/image_raw' in topics:
                    sensor_status["camera"] = True
                else:
                    issues.append("Camera topic not found")
                    
                if '/barlas/sensors/gps' in topics:
                    sensor_status["gps"] = True
                else:
                    issues.append("GPS topic not found")
                    
            except Exception as e:
                issues.append(f"Topic check failed: {e}")
        else:
            issues.append("ROS not available for sensor check")
            
        # Fiziksel sensör kontrolü
        sensor_files = {
            "lidar": "/dev/ttyUSB0",
            "imu": "/dev/i2c-1", 
            "camera": "/dev/video0"
        }
        
        for sensor, device_file in sensor_files.items():
            if os.path.exists(device_file):
                self.logger.debug(f"{sensor} device file exists: {device_file}")
            else:
                issues.append(f"{sensor} device not found: {device_file}")
                
        # Sonuç değerlendirmesi
        healthy_sensors = sum(sensor_status.values())
        total_sensors = len(sensor_status)
        
        if healthy_sensors == total_sensors:
            status = "PASS"
            details = "All sensors healthy"
        elif healthy_sensors >= total_sensors * 0.7:  # %70 sağlıklı
            status = "WARNING" 
            details = f"Some sensor issues: {', '.join(issues)}"
        else:
            status = "FAIL"
            details = f"Critical sensor failures: {', '.join(issues)}"
            
        return TestResult("Sensor Health", status, 0.0, details)
        
    def _test_competition_modules(self) -> TestResult:
        """Yarışma modülü testi"""
        required_modules = [
            "competition_manager.py",
            "traffic_sign_recognition.py", 
            "parkour_task_manager.py"
        ]
        
        issues = []
        
        for module in required_modules:
            module_path = f"/src/{module}"  # Workspace relative path
            full_path = f"d:\\barlas\\src\\{module}"
            
            if os.path.exists(full_path):
                # Modül import testi
                try:
                    module_name = module.replace('.py', '')
                    sys.path.insert(0, "d:\\barlas\\src")
                    __import__(module_name)
                    self.logger.debug(f"Successfully imported {module_name}")
                except ImportError as e:
                    issues.append(f"Import failed for {module}: {e}")
                except Exception as e:
                    issues.append(f"Module error {module}: {e}")
            else:
                issues.append(f"Module not found: {module}")
                
        if not issues:
            return TestResult("Competition Modules", "PASS", 0.0, "All competition modules available")
        else:
            return TestResult("Competition Modules", "FAIL", 0.0, f"Module issues: {', '.join(issues)}")
            
    def _test_navigation_stack(self) -> TestResult:
        """Navigasyon stack testi"""
        if not ROS_AVAILABLE:
            return TestResult("Navigation Stack", "WARNING", 0.0, "ROS not available for nav test")
            
        try:
            topics = dict(rospy.get_published_topics())
            required_topics = [
                '/barlas/control/cmd_vel',
                '/barlas/sensors/scan',
                '/barlas/navigation/local_path'
            ]
            
            missing_topics = [topic for topic in required_topics if topic not in topics]
            
            if missing_topics:
                return TestResult("Navigation Stack", "WARNING", 0.0, 
                                f"Missing nav topics: {missing_topics}")
            else:
                return TestResult("Navigation Stack", "PASS", 0.0, "Navigation topics available")
                
        except Exception as e:
            return TestResult("Navigation Stack", "FAIL", 0.0, f"Navigation check failed: {e}")
            
    def _test_control_systems(self) -> TestResult:
        """Kontrol sistemi testi"""
        control_files = [
            "pid_controller.py",
            "motor_driver.py",
            "barlas_hibrid_controller.py"
        ]
        
        issues = []
        
        for control_file in control_files:
            file_path = f"d:\\barlas\\src\\{control_file}"
            if not os.path.exists(file_path):
                issues.append(f"Control file missing: {control_file}")
            else:
                # Dosya boyut kontrolü
                if os.path.getsize(file_path) < 100:  # Çok küçük dosya
                    issues.append(f"Control file too small: {control_file}")
                    
        if not issues:
            return TestResult("Control Systems", "PASS", 0.0, "All control files present")
        else:
            return TestResult("Control Systems", "WARNING", 0.0, f"Control issues: {', '.join(issues)}")
            
    def _test_safety_systems(self) -> TestResult:
        """Güvenlik sistemi testi"""
        safety_checks = []
        
        # Emergency stop kontrolü
        if ROS_AVAILABLE:
            try:
                topics = dict(rospy.get_published_topics())
                if '/barlas/safety/emergency_stop' in topics:
                    safety_checks.append("Emergency stop topic available")
                else:
                    safety_checks.append("Emergency stop topic missing")
            except:
                safety_checks.append("Safety topic check failed")
        
        # Battery monitor kontrolü
        battery_file = "d:\\barlas\\src\\sensors\\battery_sensor.py"
        if os.path.exists(battery_file):
            safety_checks.append("Battery monitor available")
        else:
            safety_checks.append("Battery monitor missing")
            
        # Temperature monitor kontrolü  
        temp_file = "d:\\barlas\\src\\sensors\\temperature_sensor.py"
        if os.path.exists(temp_file):
            safety_checks.append("Temperature monitor available")
        else:
            safety_checks.append("Temperature monitor missing")
            
        failures = [check for check in safety_checks if "missing" in check or "failed" in check]
        
        if not failures:
            return TestResult("Safety Systems", "PASS", 0.0, "Safety systems operational")
        else:
            return TestResult("Safety Systems", "WARNING", 0.0, f"Safety issues: {', '.join(failures)}")
            
    def _test_communication(self) -> TestResult:
        """İletişim sistemi testi"""
        comm_tests = []
        
        # MAVROS bağlantısı
        if ROS_AVAILABLE:
            try:
                topics = dict(rospy.get_published_topics())
                mavros_topics = [t for t in topics.keys() if 'mavros' in t.lower()]
                if mavros_topics:
                    comm_tests.append(f"MAVROS active ({len(mavros_topics)} topics)")
                else:
                    comm_tests.append("MAVROS topics not found")
            except:
                comm_tests.append("MAVROS check failed")
        
        # ROS bridge kontrolü
        bridge_file = "d:\\barlas\\src\\barlas_ros_bridge.py"
        if os.path.exists(bridge_file):
            comm_tests.append("ROS bridge available")
        else:
            comm_tests.append("ROS bridge missing")
            
        failures = [test for test in comm_tests if "missing" in test or "failed" in test or "not found" in test]
        
        if not failures:
            return TestResult("Communication", "PASS", 0.0, "Communication systems operational")
        else:
            return TestResult("Communication", "WARNING", 0.0, f"Comm issues: {', '.join(failures)}")
            
    def _test_file_system(self) -> TestResult:
        """Dosya sistemi testi"""
        critical_paths = [
            "d:\\barlas\\src",
            "d:\\barlas\\launch", 
            "d:\\barlas\\config",
            "d:\\barlas\\dataset",
            "d:\\barlas\\models"
        ]
        
        issues = []
        
        for path in critical_paths:
            if not os.path.exists(path):
                issues.append(f"Critical path missing: {path}")
            elif not os.access(path, os.R_OK):
                issues.append(f"Path not readable: {path}")
                
        # Disk space kontrolü
        try:
            import shutil
            disk_usage = shutil.disk_usage("d:\\barlas")
            free_gb = disk_usage.free / (1024**3)
            if free_gb < 1.0:  # 1GB altında
                issues.append(f"Low disk space: {free_gb:.1f}GB free")
        except:
            issues.append("Disk space check failed")
            
        if not issues:
            return TestResult("File System", "PASS", 0.0, "File system healthy")
        else:
            status = "FAIL" if any("missing" in issue for issue in issues) else "WARNING"
            return TestResult("File System", status, 0.0, f"FS issues: {', '.join(issues)}")
            
    def _test_dependencies(self) -> TestResult:
        """Bağımlılık testi"""
        critical_imports = [
            ("numpy", "np"),
            ("opencv-python", "cv2"),
            ("yaml", "yaml"),
            ("serial", "serial"),
            ("threading", "threading")
        ]
        
        missing_deps = []
        
        for package, import_name in critical_imports:
            try:
                __import__(import_name)
            except ImportError:
                missing_deps.append(package)
                
        if not missing_deps:
            return TestResult("Dependencies", "PASS", 0.0, "All critical dependencies available")
        else:
            return TestResult("Dependencies", "FAIL", 0.0, f"Missing dependencies: {', '.join(missing_deps)}")
            
    def _generate_diagnostic_report(self):
        """Tanı raporu üret"""
        report = {
            "timestamp": datetime.now().isoformat(),
            "total_tests": len(self.test_results),
            "passed": len([r for r in self.test_results if r.status == "PASS"]),
            "warnings": len([r for r in self.test_results if r.status == "WARNING"]),
            "failed": len([r for r in self.test_results if r.status == "FAIL"]),
            "results": [asdict(result) for result in self.test_results]
        }
        
        # Raporu dosyaya yaz
        report_file = f"/tmp/barlas_debug/diagnostic_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        try:
            with open(report_file, 'w') as f:
                json.dump(report, f, indent=2)
            self.logger.info(f"Diagnostic report saved to: {report_file}")
        except Exception as e:
            self.logger.error(f"Failed to save diagnostic report: {e}")
            
        # Console özeti
        self.logger.info("=== DIAGNOSTIC SUMMARY ===")
        self.logger.info(f"Total Tests: {report['total_tests']}")
        self.logger.info(f"Passed: {report['passed']}")
        self.logger.info(f"Warnings: {report['warnings']}")
        self.logger.info(f"Failed: {report['failed']}")
        
        for result in self.test_results:
            status_icon = {"PASS": "✓", "WARNING": "⚠", "FAIL": "✗"}
            self.logger.info(f"{status_icon.get(result.status, '?')} {result.test_name}: {result.details}")
            
    def start_performance_monitoring(self, interval=10.0):
        """Performans izleme başlat"""
        if self.is_monitoring:
            self.logger.warning("Performance monitoring already running")
            return
            
        self.is_monitoring = True
        self.monitoring_thread = threading.Thread(target=self._performance_monitor_loop, args=(interval,))
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        self.logger.info(f"Performance monitoring started (interval: {interval}s)")
        
    def stop_performance_monitoring(self):
        """Performans izleme durdur"""
        self.is_monitoring = False
        self.logger.info("Performance monitoring stopped")
        
    def _performance_monitor_loop(self, interval):
        """Performans izleme döngüsü"""
        while self.is_monitoring:
            try:
                metrics = self._collect_system_metrics()
                self.metrics_history.append(metrics)
                
                # History'yi sınırla (son 1000 entry)
                if len(self.metrics_history) > 1000:
                    self.metrics_history = self.metrics_history[-1000:]
                    
                # Kritik durumları kontrol et
                self._check_critical_conditions(metrics)
                
            except Exception as e:
                self.logger.error(f"Performance monitoring error: {e}")
                
            time.sleep(interval)
            
    def _collect_system_metrics(self) -> SystemMetrics:
        """Sistem metriklerini topla"""
        import psutil
        
        # Sistem metrikleri
        cpu_usage = psutil.cpu_percent(interval=1)
        memory = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        
        # Sensör sağlığı (mock data for now)
        sensor_health = {
            "lidar": True,
            "imu": True,
            "ultrasonic": True,
            "camera": True,
            "gps": True
        }
        
        return SystemMetrics(
            timestamp=time.time(),
            cpu_usage=cpu_usage,
            memory_usage=memory.percent,
            disk_usage=disk.percent,
            network_latency=0.0,  # TODO: Implement ping test
            sensor_health=sensor_health,
            competition_status="unknown",
            current_task="monitoring",
            score=0
        )
        
    def _check_critical_conditions(self, metrics: SystemMetrics):
        """Kritik durumları kontrol et"""
        warnings = []
        
        if metrics.cpu_usage > 90:
            warnings.append(f"High CPU usage: {metrics.cpu_usage}%")
            
        if metrics.memory_usage > 90:
            warnings.append(f"High memory usage: {metrics.memory_usage}%")
            
        if metrics.disk_usage > 90:
            warnings.append(f"High disk usage: {metrics.disk_usage}%")
            
        # Unhealthy sensors
        unhealthy = [sensor for sensor, healthy in metrics.sensor_health.items() if not healthy]
        if unhealthy:
            warnings.append(f"Unhealthy sensors: {', '.join(unhealthy)}")
            
        for warning in warnings:
            self.logger.warning(f"CRITICAL: {warning}")
            
    def generate_system_report(self) -> str:
        """Kapsamlı sistem raporu üret"""
        self.logger.info("Generating comprehensive system report...")
        
        # Diagnostics çalıştır
        self.run_system_diagnostics()
        
        # Performans verilerini analiz et
        if self.metrics_history:
            avg_cpu = np.mean([m.cpu_usage for m in self.metrics_history[-100:]])  # Son 100 ölçüm
            avg_memory = np.mean([m.memory_usage for m in self.metrics_history[-100:]])
        else:
            avg_cpu = avg_memory = 0.0
            
        # Rapor formatı
        report_lines = [
            "=" * 50,
            "BARLAS SISTEM RAPORU",
            f"Oluşturma Zamanı: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            "=" * 50,
            "",
            "GENEL DURUM:",
            f"- Toplam Test: {len(self.test_results)}",
            f"- Başarılı: {len([r for r in self.test_results if r.status == 'PASS'])}",
            f"- Uyarı: {len([r for r in self.test_results if r.status == 'WARNING'])}",
            f"- Hata: {len([r for r in self.test_results if r.status == 'FAIL'])}",
            "",
            "PERFORMANS:",
            f"- Ortalama CPU: {avg_cpu:.1f}%",
            f"- Ortalama Memory: {avg_memory:.1f}%",
            "",
            "DETAYLAR:"
        ]
        
        # Test sonuçlarını ekle
        for result in self.test_results:
            status_text = {"PASS": "✓ BAŞARILI", "WARNING": "⚠ UYARI", "FAIL": "✗ HATA"}[result.status]
            report_lines.extend([
                f"- {result.test_name}: {status_text}",
                f"  Detay: {result.details}",
                f"  Süre: {result.duration:.2f}s",
                ""
            ])
            
        report_content = "\n".join(report_lines)
        
        # Dosyaya kaydet
        report_file = f"/tmp/barlas_debug/system_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        try:
            os.makedirs(os.path.dirname(report_file), exist_ok=True)
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write(report_content)
            self.logger.info(f"System report saved to: {report_file}")
        except Exception as e:
            self.logger.error(f"Failed to save system report: {e}")
            
        return report_content

def main():
    """Ana debug fonksiyonu"""
    print("BARLAS Sistem Debug Araçları")
    print("=" * 40)
    
    debugger = BARLASSystemDebugger()
    
    while True:
        print("\nMevcut Seçenekler:")
        print("1. Sistem Tanı Testleri Çalıştır")
        print("2. Performans İzleme Başlat")
        print("3. Performans İzleme Durdur") 
        print("4. Sistem Raporu Üret")
        print("5. Çıkış")
        
        try:
            choice = input("\nSeçiminizi yapın (1-5): ").strip()
            
            if choice == "1":
                print("Sistem tanı testleri çalıştırılıyor...")
                results = debugger.run_system_diagnostics()
                print(f"Testler tamamlandı. {len(results)} test çalıştırıldı.")
                
            elif choice == "2":
                interval = input("İzleme aralığı (saniye, varsayılan 10): ").strip()
                interval = float(interval) if interval else 10.0
                debugger.start_performance_monitoring(interval)
                
            elif choice == "3":
                debugger.stop_performance_monitoring()
                
            elif choice == "4":
                report = debugger.generate_system_report()
                print("Sistem raporu üretildi:")
                print(report[:1000] + "..." if len(report) > 1000 else report)
                
            elif choice == "5":
                debugger.stop_performance_monitoring()
                print("Debug araçları kapatılıyor...")
                break
                
            else:
                print("Geçersiz seçim. Lütfen 1-5 arasında bir sayı girin.")
                
        except KeyboardInterrupt:
            debugger.stop_performance_monitoring()
            print("\nÇıkış yapılıyor...")
            break
        except Exception as e:
            print(f"Hata: {e}")

if __name__ == "__main__":
    main()
