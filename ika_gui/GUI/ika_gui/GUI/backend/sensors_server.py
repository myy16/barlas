# backend/sensors_server.py
from flask import Flask, jsonify
import time
# eğer gerçek sensör driver'ların varsa import et burada (rplidar, RPi.GPIO, smbus2 vb.)

app = Flask(__name__)

# DUMMY placeholders — entegrasyon sırasında bunları gerçek sensör okumaları ile değiştir
state = {
    "lidar_min": 1.23,        # metre
    "ultrasonic_cm": 120.0,   # cm
    "imu": {"x": 0.01, "y": -0.02, "z": 9.81},
    "lat": None,
    "lon": None
}

@app.route('/sensors', methods=['GET'])
def sensors():
    # burada gerçek sensör okumalarını güncelle
    # örn: state['lidar_min'] = get_lidar_min()
    return jsonify(state)

if __name__ == "__main__":
    # production için gunicorn/nginx öneririm; test için Flask builtin server yeter
    app.run(host='0.0.0.0', port=5000)
