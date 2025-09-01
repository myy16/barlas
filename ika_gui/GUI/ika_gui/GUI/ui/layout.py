# ui/layout.py
from tkinter import ttk, StringVar, LabelFrame, Label, Radiobutton, Button
from .colors import RENK_BAGLANTI_VAR, RENK_BAGLANTI_YOK
from control.controller import VehicleController

def build_gui(root):
    # ====== burayı sistemine göre düzenle ======
    # Windows örnek: port="COM3"
    # Linux örnek: port="/dev/ttyUSB0"
    controller = VehicleController(
        port="COM3",
        baudrate=9600,
        pixhawk_url=None,    # örn "udp:127.0.0.1:14550" veya None
        rpi_url=None,        # örn "http://192.168.1.50:5000" veya None
        use_pixhawk=False,
        use_rpi=False
    )
    # ===========================================

    mod_var = StringVar(value="Manuel")

    # === MOD SEÇİMİ ===
    mod_frame = LabelFrame(root, text="MOD SEÇİMİ", padx=10, pady=5)
    mod_frame.pack(fill="x", padx=10, pady=5)
    Radiobutton(mod_frame, text="Manuel", variable=mod_var, value="Manuel",
                command=lambda: guncelle_mod()).pack(side="left", padx=10)
    Radiobutton(mod_frame, text="Otonom", variable=mod_var, value="Otonom",
                command=lambda: guncelle_mod()).pack(side="left", padx=10)

    # Hız
    hiz_frame = LabelFrame(root, text="HIZ (km/h)", padx=10, pady=5)
    hiz_label = Label(hiz_frame, text="0.0 km/h")
    hiz_label.pack()

    # Heading
    yon_frame = LabelFrame(root, text="YÖN / HEADING", padx=10, pady=5)
    yon_label = Label(yon_frame, text="Duruyor")
    yon_label.pack()

    # Konum
    konum_frame = LabelFrame(root, text="LOKASYON (GPS)", padx=10, pady=5)
    konum_label = Label(konum_frame, text="Enlem: 0.000000, Boylam: 0.000000")
    konum_label.pack()

    # Manuel kontrol
    kontrol_frame = LabelFrame(root, text="MANUEL KONTROL (Arduino)", padx=10, pady=5)
    Button(kontrol_frame, text="İleri", command=lambda: controller.forward()).pack(side="left", padx=5)
    Button(kontrol_frame, text="Geri", command=lambda: controller.backward()).pack(side="left", padx=5)
    Button(kontrol_frame, text="Sol", command=lambda: controller.left()).pack(side="left", padx=5)
    Button(kontrol_frame, text="Sağ", command=lambda: controller.right()).pack(side="left", padx=5)
    Button(kontrol_frame, text="Dur", command=lambda: controller.stop()).pack(side="left", padx=5)

    # Sensör verileri
    sensor_frame = LabelFrame(root, text="SENSÖR VERİLERİ", padx=10, pady=5)
    imu_x_label = Label(sensor_frame, text="IMU ax: 0.00"); imu_x_label.pack(anchor="w")
    imu_y_label = Label(sensor_frame, text="IMU ay: 0.00"); imu_y_label.pack(anchor="w")
    imu_z_label = Label(sensor_frame, text="IMU az: 0.00"); imu_z_label.pack(anchor="w")
    lidar_label = Label(sensor_frame, text="LiDAR min: -"); lidar_label.pack(anchor="w")
    ultra_label = Label(sensor_frame, text="Ultrasonik: - cm"); ultra_label.pack(anchor="w")
    enc_label = Label(sensor_frame, text="Encoder L/R: - / -"); enc_label.pack(anchor="w")

    # Harita placeholder
    harita_frame = LabelFrame(root, text="HARİTA / YOL İZİ", padx=10, pady=5)
    Label(harita_frame, text="(İleride çizim/plot eklenecek)").pack()

    # Bağlantı durumları
    baglanti_frame = LabelFrame(root, text="BAĞLANTI DURUMU", padx=10, pady=5)
    baglanti_label = Label(baglanti_frame, text="Seri: Yok", bg=RENK_BAGLANTI_YOK, fg="white")
    baglanti_label.pack(fill="x", pady=2)
    pix_label = Label(baglanti_frame, text="Pixhawk: Pasif", bg=RENK_BAGLANTI_YOK, fg="white")
    pix_label.pack(fill="x", pady=2)
    rpi_label = Label(baglanti_frame, text="RPi: Pasif", bg=RENK_BAGLANTI_YOK, fg="white")
    rpi_label.pack(fill="x", pady=2)

    # mod güncelleme
    def guncelle_mod():
        for frame in [hiz_frame, yon_frame, konum_frame, kontrol_frame, sensor_frame, harita_frame, baglanti_frame]:
            frame.pack_forget()
        hiz_frame.pack(fill="x", padx=10, pady=5)
        yon_frame.pack(fill="x", padx=10, pady=5)
        konum_frame.pack(fill="x", padx=10, pady=5)
        if mod_var.get() == "Manuel":
            kontrol_frame.pack(fill="x", padx=10, pady=5)
            sensor_frame.pack(fill="x", padx=10, pady=5)
        else:
            sensor_frame.pack(fill="x", padx=10, pady=5)
            harita_frame.pack(fill="x", padx=10, pady=5)
        baglanti_frame.pack(fill="x", padx=10, pady=5)

    # UI update loop
    def gui_guncelle():
        lat, lon = controller.get_gps()
        hiz = controller.get_speed()
        heading = controller.get_heading()
        imu_x, imu_y, imu_z = controller.get_imu()
        lidar_min = controller.get_lidar_min()
        ultra_cm = controller.get_ultrasonic_cm()
        enc_l, enc_r = controller.get_encoders()

        # update labels (safe formatting)
        try:
            hiz_label.config(text=f"{hiz:.1f} km/h")
        except:
            hiz_label.config(text=f"{hiz} km/h")
        try:
            konum_label.config(text=f"Enlem: {lat:.6f}, Boylam: {lon:.6f}")
        except:
            konum_label.config(text=f"Enlem: {lat}, Boylam: {lon}")

        if hiz > 0.3:
            yon_label.config(text=f"Heading: {heading:.1f}°  (Hareket)")
        else:
            yon_label.config(text=f"Heading: {heading:.1f}°  (Duruyor)")

        imu_x_label.config(text=f"IMU ax: {imu_x:.2f}")
        imu_y_label.config(text=f"IMU ay: {imu_y:.2f}")
        imu_z_label.config(text=f"IMU az: {imu_z:.2f}")

        lidar_label.config(text=f"LiDAR min: {lidar_min:.2f} m" if lidar_min is not None else "LiDAR min: -")
        ultra_label.config(text=f"Ultrasonik: {ultra_cm:.1f} cm" if ultra_cm is not None else "Ultrasonik: - cm")
        if enc_l is not None or enc_r is not None:
            enc_label.config(text=f"Encoder L/R: {enc_l} / {enc_r}")
        else:
            enc_label.config(text="Encoder L/R: - / -")

        # bağlantı göstergeleri
        if controller.is_connected():
            baglanti_label.config(text="Seri: Var", bg=RENK_BAGLANTI_VAR)
        else:
            baglanti_label.config(text="Seri: Yok", bg=RENK_BAGLANTI_YOK)

        if controller.is_pixhawk_connected():
            pix_label.config(text="Pixhawk: Var", bg=RENK_BAGLANTI_VAR)
        else:
            pix_label.config(text="Pixhawk: Pasif", bg=RENK_BAGLANTI_YOK)

        if controller.is_rpi_connected():
            rpi_label.config(text="RPi: Var", bg=RENK_BAGLANTI_VAR)
        else:
            rpi_label.config(text="RPi: Pasif", bg=RENK_BAGLANTI_YOK)

        root.after(500, gui_guncelle)

    guncelle_mod()
    gui_guncelle()
