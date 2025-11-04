#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
import numpy as np
import time


class RefPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_ref_publisher")

        # --------------------- Publicadores ---------------------
        self.pub_ref = self.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)
        self.pub_marker = self.create_publisher(Marker, "/ref_marker", 10)

        # --------------------- Parámetros de la trayectoria ---------------------
        self.dt = 1.0 / 30.0  # 30 Hz
        self.T_total = 40.0   # Duración total de la trayectoria (s)
        self.omega = 2 * np.pi / self.T_total
        self.t0 = time.time()

        # Variables auxiliares
        self.first_sample = True
        self.last_yaw = 0.0

        # --------------------- Temporizador ---------------------
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f"Publicando trayectoria tipo 8 (yaw continuo) con marcador RViz "
            f"({self.T_total:.0f} s, {1/self.dt:.0f} Hz)"
        )

    # ===========================================================
    # Callback periódico
    # ===========================================================
    def timer_callback(self):
        t = time.time() - self.t0
        if t > self.T_total:
            self.get_logger().info("Trayectoria completada.")
            # Cancela el temporizador antes de apagar
            self.timer.cancel()
            # Usa after-shutdown seguro
            self.destroy_node()
            # No llames directamente a rclpy.shutdown() aquí
            return

        w = self.omega

        # --------------------- Trayectoria tipo 8 ---------------------
        # Lemniscata de Gerono con variación lenta en z
        x = 1.5 * np.sin(w * t)
        y = 1.5 * np.sin(w * t) * np.cos(w * t)
        z = 1.0 + 0.3 * np.sin(0.5 * w * t)

        # Derivadas (velocidades) aproximadas por diferencias finitas
        x_prev = 1.5 * np.sin(w * (t - self.dt))
        y_prev = 1.5 * np.sin(w * (t - self.dt)) * np.cos(w * (t - self.dt))
        z_prev = 1.0 + 0.3 * np.sin(0.5 * w * (t - self.dt))

        dx = (x - x_prev) / self.dt
        dy = (y - y_prev) / self.dt
        dz = (z - z_prev) / self.dt

        yaw = 0.0
        wyaw = 0.0

        # --------------------- Publicar vector de referencia ---------------------
        msg = Float64MultiArray()
        msg.data = [x, y, z, yaw, dx, dy, dz, wyaw]
        self.pub_ref.publish(msg)

        # --------------------- Publicar marcador para RViz ---------------------
        marker = Marker()
        marker.header.frame_id = "odom"  # Cambia si tu TF usa otro frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ref_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        self.pub_marker.publish(marker)


# ===============================================================
# Punto de entrada
# ===============================================================
def main():
    rclpy.init()
    node = RefPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado: cerrando nodo.")
    finally:
        # Garantiza un cierre limpio sin errores de contexto
        if rclpy.ok():
            node.get_logger().info("Apagando contexto ROS2.")
            rclpy.shutdown()


if __name__ == "__main__":
    main()
