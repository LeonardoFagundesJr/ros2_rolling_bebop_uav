#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import tkinter as tk
import subprocess
import os
import time

"""
GUI de control para Parrot Bebop 2 en ROS2.
Autor: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar)
"""

class BebopControlGUI(Node):
    def __init__(self):
        super().__init__('bebop_control_gui')

        # === ROS2 publishers ===
        self.pub_takeoff = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/bebop/land', 10)
        self.pub_velref = self.create_publisher(Twist, '/nero/vel_ref', 10)
        self.pub_cam = self.create_publisher(Vector3, '/bebop/move_camera', 10)

        # Procesos externos
        self.vel_to_ctrl_proc = None     # ros2 run nero_drone VelocityControllerBody
        self.ref_pub_proc = None         # ros2 run nero_drone RefPublisher

        self.takeoff_flag = False
        self.land_flag = True

        self.get_logger().info("Bebop Control GUI node started.")

        # === Tkinter setup ===
        self.root = tk.Tk()
        self.root.title("Bebop Control Panel")
        self.root.configure(bg="#2C2C2C")

        main_frame = tk.Frame(self.root, bg="#2C2C2C")
        main_frame.pack(padx=15, pady=15)

        left_frame = tk.Frame(main_frame, bg="#2C2C2C")
        left_frame.grid(row=0, column=0, padx=10)
        right_frame = tk.Frame(main_frame, bg="#2C2C2C")
        right_frame.grid(row=0, column=1, padx=10)

        # === Label de estado / conteo ===
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
        self.btn_takeoff.pack(pady=5)

        self.btn_land = tk.Button(left_frame, text="Land", command=self.land, **button_cfg)
        self.btn_hover = tk.Button(left_frame, text="Hover", command=self.hover, **button_cfg)
        self.btn_visual = tk.Button(left_frame, text="Visual Control", command=self.visual_control, **button_cfg)

        self.btn_hover.pack(pady=5)
        self.btn_visual.pack(pady=5)
        self.btn_land.pack(pady=5)

        # === Cámara (slider) ===
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

        # Estado inicial
        self.disable_all_buttons()
        self.btn_takeoff.config(state=tk.NORMAL)
        self.send_camera_angle(-15.0)
        self.get_logger().info("Initial camera angle set to -15.0°")

    # === Drone controls ===
    def takeoff(self):
        self.pub_takeoff.publish(Empty())
        self.get_logger().info("Takeoff command sent.")
        self.takeoff_flag = True
        self.land_flag = False
        self.disable_all_buttons()
        self.countdown(5, "Esperando estabilización", self.enable_flight_controls)

    def land(self):
        if self.vel_to_ctrl_proc and self.vel_to_ctrl_proc.poll() is None:
            self.get_logger().info("Stopping VelocityControllerBody before landing...")
            self.vel_to_ctrl_proc.terminate()
            try:
                self.vel_to_ctrl_proc.wait(timeout=2)
                self.get_logger().info("VelocityControllerBody process stopped successfully.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("VelocityControllerBody did not stop gracefully, killing process.")
                self.vel_to_ctrl_proc.kill()
            self.vel_to_ctrl_proc = None

        if self.ref_pub_proc and self.ref_pub_proc.poll() is None:
            self.get_logger().info("Stopping RefPublisher before landing...")
            self.ref_pub_proc.terminate()
            try:
                self.ref_pub_proc.wait(timeout=2)
                self.get_logger().info("RefPublisher process stopped successfully.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("RefPublisher did not stop gracefully, killing process.")
                self.ref_pub_proc.kill()
            self.ref_pub_proc = None

        self.pub_land.publish(Empty())
        self.get_logger().info("Land command sent.")

        self.takeoff_flag = False
        self.land_flag = True
        self.disable_all_buttons()
        self.countdown(10, "Aterrizando, espere", self.enable_takeoff_only)

    def hover(self):
        msg = Twist()
        msg.linear.x = msg.linear.y = msg.linear.z = 0.0
        msg.angular.z = 0.0
        self.pub_velref.publish(msg)
        self.get_logger().info("Hover command published (zeros to /nero/vel_ref).")

    # === Cámara ===
    def send_camera_angle(self, angle_deg: float):
        msg = Vector3()
        msg.x = float(angle_deg)
        msg.y = msg.z = 0.0
        self.pub_cam.publish(msg)
        self.get_logger().info(f"Camera pitch command sent: x={angle_deg:.1f}°")

    def update_camera_angle(self, val):
        self.send_camera_angle(float(val))

    # === Control modes ===
    def visual_control(self):
        self.get_logger().info("Visual control mode selected (no command sent).")

    # === Proceso de control VelocityControllerBody ===
    def start_vel_to_control(self):
        if self.vel_to_ctrl_proc is None or self.vel_to_ctrl_proc.poll() is not None:
            self.get_logger().info("Launching VelocityControllerBody process...")
            self.vel_to_ctrl_proc = subprocess.Popen(
                ["ros2", "run", "nero_drone", "VelocityControllerBody"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=os.environ.copy()
            )
        else:
            self.get_logger().info("VelocityControllerBody already running.")

    # === Control de interfaz ===
    def disable_all_buttons(self):
        for btn in [self.btn_takeoff, self.btn_land, self.btn_hover, self.btn_visual]:
            btn.config(state=tk.DISABLED)

    def enable_flight_controls(self):
        for btn in [self.btn_hover, self.btn_visual, self.btn_land]:
            btn.config(state=tk.NORMAL)
        self.status_label.config(text="")
        self.get_logger().info("Drone stabilized. Flight controls enabled.")
        self.start_vel_to_control()

    def enable_takeoff_only(self):
        self.btn_takeoff.config(state=tk.NORMAL)
        self.status_label.config(text="")
        self.slider.set(-15)
        self.send_camera_angle(-15.0)
        self.get_logger().info("Landing complete. Only Takeoff button enabled.")

    def countdown(self, seconds, message, callback):
        if seconds > 0:
            self.status_label.config(text=f"{message}: {seconds} s")
            self.root.after(1000, self.countdown, seconds - 1, message, callback)
        else:
            callback()

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.spin_once)

    def on_close(self):
        self.get_logger().info("Closing Bebop Control GUI.")
        for proc_name, proc in [("VelocityControllerBody", self.vel_to_ctrl_proc), ("RefPublisher", self.ref_pub_proc)]:
            if proc and proc.poll() is None:
                self.get_logger().info(f"Terminating {proc_name} process.")
                proc.terminate()

        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = BebopControlGUI()
    node.root.after(100, node.spin_once)
    node.root.mainloop()


if __name__ == "__main__":
    main()
