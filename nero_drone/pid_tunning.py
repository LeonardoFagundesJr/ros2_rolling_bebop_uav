#!/usr/bin/env python3
# GUI para ajustar PID no lineal (matriz 3x4) y publicar en ROS2 con historial automático
# Autor: Brayan Saldarriaga-Mesa & ChatGPT, 2025

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk, filedialog
from datetime import datetime


class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_gui_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/drone/pid_gains', 10)

    def publish_values(self, vals):
        msg = Float64MultiArray()
        msg.data = vals
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published PID gains: {", ".join([f"{v:.4f}" for v in vals])}'
        )


class PIDGui:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Drone PID Nonlinear Tuner (3x4 Matrix)")
        self.root.geometry("580x380")
        self.root.configure(bg="#1e1e1e")

        style = ttk.Style()
        style.configure("TLabel", background="#1e1e1e", foreground="white", font=("Arial", 10))
        style.configure("TButton", font=("Arial", 10, "bold"))
        style.configure("TSpinbox", font=("Consolas", 10))

        self.entries = []  # Lista lineal con 12 DoubleVar

        frame = ttk.Frame(self.root, padding=20)
        frame.pack(fill="both", expand=True)

        ttk.Label(frame, text="PID Nonlinear Gain Matrix", font=("Arial", 16, "bold")).grid(
            row=0, column=0, columnspan=5, pady=10
        )

        # Encabezados de columnas
        headers = ["", "P1", "P2", "D", "I"]
        for j, h in enumerate(headers):
            ttk.Label(frame, text=h, width=8).grid(row=1, column=j, padx=5, pady=5)

        axes = ["X", "Y", "Z"]

        # Crear 3 filas (x,y,z) × 4 columnas (p1,p2,d,i)
        for i, axis in enumerate(axes):
            ttk.Label(frame, text=axis, width=4).grid(row=i + 2, column=0, pady=4)
            for j in range(4):
                var = tk.DoubleVar(value=0.0)
                spin = ttk.Spinbox(
                    frame,
                    from_=0.0,
                    to=5.0,
                    increment=0.0001,
                    textvariable=var,
                    width=8,
                    format="%.4f"
                )
                spin.grid(row=i + 2, column=j + 1, padx=4, pady=4)
                self.entries.append(var)

        # Botones
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=6, column=0, columnspan=5, pady=15)

        ttk.Button(btn_frame, text="Enviar PID", command=self.send_to_ros).grid(row=0, column=0, padx=10)
        ttk.Button(btn_frame, text="Guardar PID", command=self.save_to_file).grid(row=0, column=1, padx=10)
        ttk.Button(btn_frame, text="Reset (solo enviar ceros)", command=self.reset_to_zero).grid(row=0, column=2, padx=10)

        ttk.Label(
            frame,
            text="Resolución: 0.0001  |  Rango: [0.0, 5.0]",
            font=("Arial", 9)
        ).grid(row=7, column=0, columnspan=5, pady=5)

        self.root.after(100, self.spin_once)

    def get_values(self):
        """Retorna los 12 valores en orden: [P1x, P1y, P1z, P2x, P2y, P2z, Dx, Dy, Dz, Ix, Iy, Iz]"""
        # reorganizar para enviar en orden consistente (por columnas)
        p1 = [self.entries[0].get(), self.entries[4].get(), self.entries[8].get()]  # X,Y,Z
        p2 = [self.entries[1].get(), self.entries[5].get(), self.entries[9].get()]
        d = [self.entries[2].get(), self.entries[6].get(), self.entries[10].get()]
        i = [self.entries[3].get(), self.entries[7].get(), self.entries[11].get()]
        vals = p1 + p2 + d + i
        return [round(v, 4) for v in vals]

    def send_to_ros(self):
        vals = self.get_values()
        self.node.publish_values(vals)
        self.append_to_history(vals)

    def save_to_file(self):
        vals = self.get_values()
        fname = filedialog.asksaveasfilename(
            title="Guardar PID Gains",
            defaultextension=".txt",
            filetypes=[("Archivo de texto", "*.txt")]
        )
        if fname:
            with open(fname, "w") as f:
                f.write("# PID Nonlinear Gains (3x4): P1x P1y P1z P2x P2y P2z Dx Dy Dz Ix Iy Iz\n")
                f.write(" ".join([f"{v:.4f}" for v in vals]) + "\n")

    def reset_to_zero(self):
        """Envía un vector de ceros al dron sin borrar los valores mostrados en la GUI."""
        zero_vals = [0.0] * 12
        self.node.publish_values(zero_vals)
        self.append_to_history(zero_vals)

    def append_to_history(self, vals):
        line = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  " + \
               " ".join([f"{v:.4f}" for v in vals]) + "\n"
        with open("history.txt", "a") as f:
            f.write(line)

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self.root.after(50, self.spin_once)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = PIDPublisher()
    gui = PIDGui(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
