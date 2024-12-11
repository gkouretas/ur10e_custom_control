from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

_hard_coded_args = {
    "ur_type": "ur10e",
    "robot_ip": "192.168.57.101",
    "launch_simulation": "true",
    "simulation_version": "5.9.4",
    "headless_mode": "true"
}

def generate_launch_description():
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"]),
        launch_arguments = _hard_coded_args.items()
    )

    return LaunchDescription(
        [ur_control_launch]
    )