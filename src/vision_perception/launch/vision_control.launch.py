import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory("vision_perception")
    model_path = os.path.join(pkg_share, "weights", "best.onnx")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    vision_control_node = Node(
        package="vision_perception",
        executable="vision_control_node",
        name="vision_control_node",
        output="screen",
        parameters=[
            {
                "model_path": model_path,
                "conf_threshold": 0.15,
                "iou_threshold": 0.35,
                "angular_gain": 1.0,
                "angle_tolerance": 0.05,
                "control_rate": 10.0,
                "use_sim_time": use_sim_time
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        vision_control_node
    ])
