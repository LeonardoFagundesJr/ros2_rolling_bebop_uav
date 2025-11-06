from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('nero_drone')
    urdf_file = os.path.join(pkg_share, 'urdf', 'bebop2.urdf')
    rviz_config = os.path.join(pkg_share, 'others', 'bebop2.rviz')

    return LaunchDescription([
        # --- Calibration Publisher ---
        Node(
            package='nero_drone',
            executable='tf_cam',
            name='tf_cam',
            output='screen'
        ),
        
        Node(
            package='nero_drone',
            executable='isfly',
            name='isfly',
            output='screen',
        ),       

        Node(
            package='nero_drone',
            executable='safety_watchdog',
            name='safety_watchdog',
            output='screen'
        ), 
        
        Node(
            package='nero_drone',
            executable='safe_bebop_republisher',
            name='safe_bebop_republisher',
            output='screen'
        ),

        Node(
            package='nero_drone',
            executable='inverse_dynamic_controller',
            name='inverse_dynamic_controller',
            output='screen',
        ),   

        Node(
            package='nero_drone',
            executable='bebop_control_gui.py',
            name='bebop_control_gui',
            output='screen'
        ),
    ])
