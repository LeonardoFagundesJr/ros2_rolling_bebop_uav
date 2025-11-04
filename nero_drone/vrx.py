#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import HilSensor, HilGPS
import math


class VRXToMAVROSBridge(Node):
    def __init__(self):
        super().__init__('vrx_to_mavros_bridge')

        # Publicadores hacia MAVROS
        self.pub_hil_sensor = self.create_publisher(HilSensor, '/mavros/hil/sensor', 10)
        self.pub_hil_gps = self.create_publisher(HilGPS, '/mavros/hil/gps', 10)

        # Suscripciones a VRX
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)

        self.pressure_ref = 1013.25  # hPa
        self.last_alt = 0.0
        self.get_logger().info('🚤 Nodo VRX → PX4 (MAVROS bridge) iniciado.')

    # ------------------- IMU → HIL_SENSOR -------------------
    def imu_cb(self, msg: Imu):
        hil = HilSensor()
        hil.header = msg.header

        # Aceleraciones (m/s²)
        hil.acc.x = msg.linear_acceleration.x
        hil.acc.y = msg.linear_acceleration.y
        hil.acc.z = msg.linear_acceleration.z

        # Giroscopio (rad/s)
        hil.gyro.x = msg.angular_velocity.x
        hil.gyro.y = msg.angular_velocity.y
        hil.gyro.z = msg.angular_velocity.z

        # Magnetómetro sintético (dirección de yaw)
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        hil.mag.x = math.cos(yaw)
        hil.mag.y = math.sin(yaw)
        hil.mag.z = 0.0

        # Barómetro sintético
        hil.abs_pressure = self.alt_to_pressure(self.last_alt or 0.0)
        hil.diff_pressure = 0.0
        hil.pressure_alt = self.last_alt
        hil.temperature = 25.0
        hil.fields_updated = 0b111111

        self.pub_hil_sensor.publish(hil)

    # ------------------- GPS → HIL_GPS -------------------
    def gps_cb(self, msg: NavSatFix):
        gps = HilGPS()
        gps.header = msg.header

        gps.fix_type = 3 if msg.status.status >= 0 else 0
        gps.geo.latitude = msg.latitude
        gps.geo.longitude = msg.longitude
        gps.geo.altitude = msg.altitude

        # Precisión (centímetros convertidos a entero)
        gps.eph = int(50)     # ≈0.5 m
        gps.epv = int(100)    # ≈1 m
        gps.vel = int(0)
        gps.vn = 0
        gps.ve = 0
        gps.vd = 0
        gps.cog = 0
        gps.satellites_visible = 10
        self.last_alt = msg.altitude

        self.pub_hil_gps.publish(gps)

    # -------------------------------------------------------
    def quaternion_to_euler(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x**2 + q.y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def alt_to_pressure(self, altitude_m):
        """Presión atmosférica estimada (hPa) según modelo ISA"""
        return 1013.25 * (1 - 2.25577e-5 * altitude_m) ** 5.2559


def main():
    rclpy.init()
    node = VRXToMAVROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando puente VRX→PX4.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

