#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
import tkinter as tk
import subprocess
import os
import signal
import re

"""
GUI simplificada para Parrot Bebop 2 en ROS2.
Contiene solo Takeoff, Land y control de cámara.
Al presionar Land:
1. Detiene los nodos que publican en /bebop/cmd_vel.
2. Envía comando de aterrizaje.
3. Espera el conteo antes de habilitar Takeoff nuevamente.
Autor: Brayan Saldarriaga-Mesa
"""

class BebopControlGUI(Node):
    def __init__(self):
        super().__init__('bebop_control_gui_minimal')

        # === ROS2 publishers ===
        self.pub_takeoff = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/bebop/land', 10)
        self.pub_cam = self.create_publisher(Vector3, '/bebop/move_camera', 10)

        self.takeoff_flag = False
        self.land_flag = True

        self.get_logger().info("Minimal Bebop Control GUI node started.")

        # === Tkinter GUI ===
        self.root = tk.Tk()
        self.root.title("Bebop Minimal Control Panel")
        self.root.configure(bg="#2C2C2C")

        main_frame = tk.Frame(self.root, bg="#2C2C2C")
        main_frame.pack(padx=15, pady=15)

        left_frame = tk.Frame(main_frame, bg="#2C2C2C")
        left_frame.grid(row=0, column=0, padx=10)
        right_frame = tk.Frame(main_frame, bg="#2C2C2C")
        right_frame.grid(row=0, column=1, padx=10)

        # === Estado ===
        self.status_label = tk.Label(left_frame, text="", font=("Arial", 12), bg="#2C2C2C", fg="yellow")
        self.status_label.pack(pady=5)

        # === Configuración de botones ===
        button_cfg = {
            "font": ("Arial", 14),
            "width": 20,
            "height": 2,
            "bg": "#3E3E3E",
            "fg": "white",
            "relief": tk.RAISED,
        }

        # === Botones ===
        self.btn_takeoff = tk.Button(left_frame, text="Takeoff", command=self.takeoff, **button_cfg)
        self.btn_takeoff.pack(pady=10)

        self.btn_land = tk.Button(left_frame, text="Land", command=self.land, **button_cfg)
        self.btn_land.pack(pady=10)

        # === Cámara ===
        tk.Label(right_frame, text="Camera Pitch (°)", font=("Arial", 12), bg="#2C2C2C", fg="white").pack(pady=5)
        self.slider = tk.Scale(
            right_frame,
            from_=15, to=-90,
            orient=tk.VERTICAL,
            length=300,
            resolution=15,
            tickinterval=15,
            font=("Arial", 10),
            bg="#2C2C2C",
            fg="white",
            highlightthickness=0,
            command=self.update_camera_angle
        )
        self.slider.set(-15)
        self.slider.pack(pady=10)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.spin_once)

        # Estado inicial
        self.disable_all_buttons()
        self.btn_takeoff.config(state=tk.NORMAL)
        self.send_camera_angle(-15.0)
        self.get_logger().info("Initial camera angle set to -15.0°")

    # === Drone controls ===
    def takeoff(self):
        self.pub_takeoff.publish(Empty())
        self.status_label.config(text="Despegando...")
        self.get_logger().info("Takeoff command sent.")
        self.takeoff_flag = True
        self.land_flag = False
        self.disable_all_buttons()
        self.countdown(5, "Esperando estabilización", self.enable_land_only)

    def land(self):
        self.disable_all_buttons()
        self.status_label.config(text="Deteniendo nodos de /bebop/cmd_vel...")
        self.get_logger().info("Stopping /bebop/cmd_vel publishers before landing.")
        self.root.after(100, self.land_sequence_step1)

    # === Secuencia LAND paso a paso ===
    def land_sequence_step1(self):
        # 1. Detener los nodos publicadores
        self.stop_cmd_vel_publishers()
        self.get_logger().info("Todos los publicadores de /bebop/cmd_vel detenidos.")

        # 2. Enviar comando LAND
        self.pub_land.publish(Empty())
        self.get_logger().info("Land command sent.")
        self.status_label.config(text="Aterrizando...")

        # 3. Esperar conteo antes de reactivar
        self.countdown(10, "Aterrizando, espere", self.enable_takeoff_only)

    # === Cámara ===
    def send_camera_angle(self, angle_deg: float):
        msg = Vector3()
        msg.x = float(angle_deg)
        msg.y = msg.z = 0.0
        self.pub_cam.publish(msg)
        self.get_logger().info(f"Camera pitch command sent: {angle_deg:.1f}°")

    def update_camera_angle(self, val):
        self.send_camera_angle(float(val))

    # === Introspección y parada de nodos ===
    def stop_cmd_vel_publishers(self):
        topic = '/bebop/cmd_vel'
        pubs = self.get_publishers_info_by_topic(topic)
        if not pubs:
            self.get_logger().info("No hay nodos publicando en /bebop/cmd_vel.")
            return

        for pub in pubs:
            node_name = pub.node_name
            ns = pub.node_namespace
            full_name = f"{ns}/{node_name}" if ns != "/" else node_name
            self.get_logger().info(f"Intentando detener nodo publicador: {full_name}")

            try:
                info = subprocess.check_output(["ros2", "node", "info", full_name], text=True)
                pid_match = re.search(r'PID:\s*(\d+)', info)
                if pid_match:
                    pid = int(pid_match.group(1))
                    os.kill(pid, signal.SIGTERM)
                    self.get_logger().info(f"Nodo {full_name} detenido (pid={pid}).")
                else:
                    self.get_logger().warn(f"No se pudo determinar el PID de {full_name}.")
            except subprocess.CalledProcessError:
                self.get_logger().warn(f"No se pudo obtener info del nodo {full_name}.")

    # === Control de botones y estado ===
    def disable_all_buttons(self):
        for btn in [self.btn_takeoff, self.btn_land]:
            btn.config(state=tk.DISABLED)

    def enable_land_only(self):
        self.btn_land.config(state=tk.NORMAL)
        self.status_label.config(text="")
        self.get_logger().info("Drone estabilizado. Habilitado solo Land.")

    def enable_takeoff_only(self):
        self.btn_takeoff.config(state=tk.NORMAL)
        self.status_label.config(text="")
        self.slider.set(-15)
        self.send_camera_angle(-15.0)
        self.get_logger().info("Aterrizaje completado. Habilitado solo Takeoff.")

    # === Temporizador ===
    def countdown(self, seconds, message, callback):
        if seconds > 0:
            self.status_label.config(text=f"{message}: {seconds} s")
            self.root.after(1000, self.countdown, seconds - 1, message, callback)
        else:
            callback()

    # === Integración Tkinter/ROS2 ===
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.spin_once)

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
