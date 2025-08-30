from tkinter import ttk, StringVar, LabelFrame, Label, Radiobutton, Button
from .colors import RENK_BAGLANTI_VAR, RENK_BAGLANTI_YOK
from control.controller import VehicleController

def build_gui(root):
    controller = VehicleController(port="COM3")

    mod_var = StringVar(value="Manuel")

    # === MOD SEÇİMİ ===
    mod_frame = LabelFrame(root, text="MOD SEÇİMİ", padx=10, pady=5)
    mod_frame.pack(fill="x", padx=10, pady=5)

    Radiobutton(mod_frame, text="Manuel", variable=mod_var, value="Manuel",
                command=lambda: guncelle_mod()).pack(side="left", padx=10)
    Radiobutton(mod_frame, text="Otonom", variable=mod_var, value="Otonom",
                command=lambda: guncelle_mod()).pack(side="left", padx=10)

    # === Alt Bölümler ===
    hiz_frame = LabelFrame(root, text="HIZ BİLGİSİ", padx=10, pady=5)
    hiz_label = Label(hiz_frame, text="0 km/h")
    hiz_label.pack()

    yon_frame = LabelFrame(root, text="YÖN KONTROLÜ", padx=10, pady=5)
    yon_label = Label(yon_frame, text="Duruyor")
    yon_label.pack()

    konum_frame = LabelFrame(root, text="LOKASYON", padx=10, pady=5)
    konum_label = Label(konum_frame, text="Enlem: 0.0000, Boylam: 0.0000")
    konum_label.pack()

    Button(yon_frame, text="İleri", command=lambda: controller.forward()).pack(side="left", padx=5)
    Button(yon_frame, text="Geri", command=lambda: controller.backward()).pack(side="left", padx=5)
    Button(yon_frame, text="Sol", command=lambda: controller.left()).pack(side="left", padx=5)
    Button(yon_frame, text="Sağ", command=lambda: controller.right()).pack(side="left", padx=5)
    Button(yon_frame, text="Dur", command=lambda: controller.stop()).pack(side="left", padx=5)

    sensor_frame = LabelFrame(root, text="SENSÖR VERİLERİ", padx=10, pady=5)
    imu_x_label = Label(sensor_frame, text="IMU X: 0.0")
    imu_x_label.pack(anchor="w")
    imu_y_label = Label(sensor_frame, text="IMU Y: 0.0")
    imu_y_label.pack(anchor="w")
    imu_z_label = Label(sensor_frame, text="IMU Z: 9.8")
    imu_z_label.pack(anchor="w")

    harita_frame = LabelFrame(root, text="HARİTA / YOL İZİ", padx=10, pady=5)
    Label(harita_frame, text="Son GPS noktaları burada gösterilir.").pack()

    baglanti_frame = LabelFrame(root, text="BAĞLANTI DURUMU", padx=10, pady=5)
    baglanti_label = Label(baglanti_frame, text="Bağlantı: Yok", bg=RENK_BAGLANTI_YOK, fg="white")
    baglanti_label.pack()

    # === Mod güncelleme fonksiyonu ===
    def guncelle_mod():
        for frame in [hiz_frame, yon_frame, konum_frame, sensor_frame, harita_frame, baglanti_frame]:
            frame.pack_forget()

        hiz_frame.pack(fill="x", padx=10, pady=5)
        yon_frame.pack(fill="x", padx=10, pady=5)
        konum_frame.pack(fill="x", padx=10, pady=5)

        if mod_var.get() == "Manuel":
            sensor_frame.pack(fill="x", padx=10, pady=5)
        else:
            harita_frame.pack(fill="x", padx=10, pady=5)

        baglanti_frame.pack(fill="x", padx=10, pady=5)

    # === Sensör verilerini GUI'de güncelle ===
    def gui_guncelle():
        controller.read_serial()

        lat, lon = controller.get_gps()
        hiz = controller.get_speed()
        imu_x, imu_y, imu_z = controller.get_imu()

        hiz_label.config(text=f"{hiz:.1f} km/h")
        konum_label.config(text=f"Enlem: {lat:.4f}, Boylam: {lon:.4f}")
        imu_x_label.config(text=f"IMU X: {imu_x:.2f}")
        imu_y_label.config(text=f"IMU Y: {imu_y:.2f}")
        imu_z_label.config(text=f"IMU Z: {imu_z:.2f}")

        if controller.is_connected():
            baglanti_label.config(text="Bağlantı: Var", bg=RENK_BAGLANTI_VAR)
        else:
            baglanti_label.config(text="Bağlantı: Yok", bg=RENK_BAGLANTI_YOK)

        # Kendini tekrar çağır (500 ms sonra)
        root.after(500, gui_guncelle)

    # Başlangıçta ayarla ve güncelleme döngüsünü başlat
    guncelle_mod()
    gui_guncelle()
