#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ActuatorControl
from std_msgs.msg import Float32

class PX4ToVRXBridge(Node):
    def __init__(self):
        super().__init__('px4_to_vrx_bridge')

        # Suscripción a los comandos de PX4 (publicados por MAVROS)
        self.create_subscription(ActuatorControl, '/mavros/hil/actuator_controls', self.cb_controls, 10)

        # Publicadores hacia los propulsores de VRX
        self.pub_left = self.create_publisher(Float32, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float32, '/wamv/thrusters/right/thrust', 10)

        # Escala de empuje — ajusta según el modelo del WAM-V
        self.thrust_scale = 100.0  # Newtons por unidad de control normalizado [-1, 1]

        self.get_logger().info('⚙️  Nodo PX4 → VRX (actuator bridge) iniciado.')

    def cb_controls(self, msg: ActuatorControl):
        # PX4 publica 8 canales (float32), tomamos los dos primeros
        left_cmd = float(msg.controls[0])
        right_cmd = float(msg.controls[1])

        # Escalar a Newtons
        left_thrust = Float32()
        right_thrust = Float32()
        left_thrust.data = self.thrust_scale * left_cmd
        right_thrust.data = self.thrust_scale * right_cmd

        # Publicar al simulador VRX
        self.pub_left.publish(left_thrust)
        self.pub_right.publish(right_thrust)

        self.get_logger().debug(f"L:{left_thrust.data:.2f} N | R:{right_thrust.data:.2f} N")

def main():
    rclpy.init()
    node = PX4ToVRXBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando puente PX4→VRX.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

