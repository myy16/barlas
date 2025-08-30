import random

def lora_verisi_al():
    """
    LoRa bağlantısı varsa, bu fonksiyon sahte veri döner.
    Gerçek sistemde burası serial veya socket üzerinden veri alır.
    """
    return {
        "speed": random.randint(0, 40),  # km/h
        "direction": random.choice(["İleri", "Geri", "Sağ", "Sol"]),
        "latitude": 39.9208,
        "longitude": 32.8541,
        "imu": {"x": 0.01, "y": -0.02, "z": 9.81}
    }
