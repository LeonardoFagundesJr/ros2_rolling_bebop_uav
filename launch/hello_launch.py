#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    # === 1. Nodo MAVROS conectado al PX4 SITL ===
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {'fcu_url': 'tcp://127.0.0.1:4560'},
            {'gcs_url': 'udp://@127.0.0.1:14550'},
            {'plugin_allowlist': [
                'sys_status',
                'sys_time',
                'param',
                'command',
                'setpoint_raw',
                'hil',
                'imu',
                'gps'
            ]},
            {'conn/heartbeat_rate': 1.0},
            {'conn/system_time_rate': 1.0},
            {'conn/timeout': 10.0},
        ]
    )


    # === 3. Nodos VRX (si los usas en la simulación) ===
    vrx_node = Node(
        package='nero_drone',
        executable='vrx.py',
        name='vrx_main',
        output='screen'
    )

    inv_vrx_node = Node(
        package='nero_drone',
        executable='inv_vrx.py',
        name='vrx_inverse',
        output='screen'
    )

    # Devuelve el conjunto de nodos a lanzar
    return LaunchDescription([
        mavros_node,
        vrx_node,
        inv_vrx_node
    ])

