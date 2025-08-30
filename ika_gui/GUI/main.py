from tkinter import Tk
from ui.layout import build_gui

if __name__ == "__main__":
    root = Tk()
    root.title("Barlas İka Kontrol Paneli")
    root.geometry("1000x600")
    build_gui(root)
    root.mainloop()
