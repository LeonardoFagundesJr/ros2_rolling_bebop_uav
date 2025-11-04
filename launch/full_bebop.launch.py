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
            executable='tf_tag_bebop',
            name='tf_tag_bebop',
            output='screen'
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
        # --- Robot State Publisher ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # --- Visualization ---
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        Node(
            package='nero_drone',
            executable='bebop_control_gui.py',
            name='bebop_control_gui',
            output='screen'
        ),

        Node(
            package='nero_drone',
            executable='ref_vec_filter',
            name='ref_vec_filter',
            output='screen'
        ),
    ])
