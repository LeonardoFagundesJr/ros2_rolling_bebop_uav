#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
import tkinter as tk


"""
GUI simplificada para Parrot Bebop 2 en ROS2.
Controla Takeoff, Land y Cámara.

- Takeoff: despega.
- Land: aterriza.
- Cámara: control de pitch mediante slider.

Autor: Brayan Saldarriaga-Mesa
"""


class BebopControlGUI(Node):
    def __init__(self):
        super().__init__('bebop_control_gui_minimal')

        # === ROS2 publishers ===
        self.pub_takeoff = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/bebop/land', 10)
        self.pub_cam = self.create_publisher(Vector3, '/bebop/move_camera', 10)

        self.get_logger().info("Minimal Bebop Control GUI node started.")

        # === GUI ===
        self.root = tk.Tk()
        self.root.title("Bebop Minimal Control Panel")
        self.root.configure(bg="#2C2C2C")

        main_frame = tk.Frame(self.root, bg="#2C2C2C")
        main_frame.pack(padx=15, pady=15)

        left_frame = tk.Frame(main_frame, bg="#2C2C2C")
        left_frame.grid(row=0, column=0, padx=10)
        right_frame = tk.Frame(main_frame, bg="#2C2C2C")
        right_frame.grid(row=0, column=1, padx=10)

        # Estado
        self.status_label = tk.Label(left_frame, text="", font=("Arial", 12),
                                     bg="#2C2C2C", fg="yellow")
        self.status_label.pack(pady=5)

        # Botones principales
        button_cfg = {"font": ("Arial", 14), "width": 20, "height": 2,
                      "bg": "#3E3E3E", "fg": "white"}
        self.btn_takeoff = tk.Button(left_frame, text="Takeoff",
                                     command=self.takeoff, **button_cfg)
        self.btn_takeoff.pack(pady=10)

        self.btn_land = tk.Button(left_frame, text="Land",
                                  command=self.land, **button_cfg)
        self.btn_land.pack(pady=10)

        # Cámara
        tk.Label(right_frame, text="Camera Pitch (°)", font=("Arial", 12),
                 bg="#2C2C2C", fg="white").pack(pady=5)

        self.slider = tk.Scale(
            right_frame, from_=15, to=-90, orient=tk.VERTICAL, length=300,
            resolution=15, tickinterval=15, font=("Arial", 10),
            bg="#2C2C2C", fg="white", highlightthickness=0,
            command=self.update_camera_angle
        )
        self.slider.set(-15)
        self.slider.pack(pady=10)

        # Configuración de ventana
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.spin_once)

        # Estado inicial
        self.btn_takeoff.config(state=tk.NORMAL)
        self.btn_land.config(state=tk.NORMAL)
        self.send_camera_angle(-15.0)

    # === TAKEOFF ===
    def takeoff(self):
        self.status_label.config(text="Despegando...")
        self.get_logger().info("Enviando comando Takeoff...")
        self.pub_takeoff.publish(Empty())
        self.status_label.config(text="Takeoff publicado.")

    # === LAND ===
    def land(self):
        self.status_label.config(text="Iniciando secuencia de aterrizaje...")
        self.get_logger().info("Comando LAND publicado.")
        self.pub_land.publish(Empty())
        self.status_label.config(text="Aterrizando...")
        self.root.after(5000, self.reset_after_land)

    def reset_after_land(self):
        self.status_label.config(text="Aterrizaje completado. Listo para nuevo takeoff.")
        self.slider.set(-15)
        self.send_camera_angle(-15.0)

    # === Cámara ===
    def send_camera_angle(self, angle_deg: float):
        msg = Vector3()
        msg.x = float(angle_deg)
        msg.y = msg.z = 0.0
        self.pub_cam.publish(msg)

    def update_camera_angle(self, val):
        self.send_camera_angle(float(val))

    # === ROS2 loop ===
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.spin_once)

    # === Cierre limpio ===
    def on_close(self):
        self.get_logger().info("Cerrando GUI y nodo ROS2.")
        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    gui = BebopControlGUI()
    gui.root.mainloop()


if __name__ == "__main__":
    main()
