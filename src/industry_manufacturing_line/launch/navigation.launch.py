import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('industry_manufacturing_line')

    urdf_path = os.path.join(pkg_share, 'config/robot/robot_description.urdf')
    world_path = os.path.join(pkg_share, 'config/world/factory.world')
    nav2_params_path = os.path.join(pkg_share, 'config/nav2/nav2_params.yaml')
    default_map_path = os.path.join(pkg_share, 'config/nav2/map.yaml')
    rviz_config = os.path.join(pkg_share, 'config/nav2_default.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_file = LaunchConfiguration('map', default=default_map_path)

    ign_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '--verbose'],
        output='screen'
    )

    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock]gz.msgs.Clock',
        '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan]gz.msgs.LaserScan',
    ]
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_topics,
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot",
            "-x", "0.0", "-y", "0.0", "-z", "0.1"
        ],
        output="screen"
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # ========== 修正：添加 joint_state_publisher ==========
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'autostart': autostart
        }.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map_path,
                              description='Full path to map yaml file'),
        ign_sim,
        gz_bridge,
        spawn_robot,
        robot_state_publisher,
        joint_state_publisher,
        nav2_bringup,
        rviz2,
    ])