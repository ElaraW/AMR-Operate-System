import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('industry_manufacturing_line')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_path = os.path.join(pkg_share, 'config', 'robot', 'robot_description.urdf')
    semantic_path = os.path.join(pkg_share, 'config', 'moveit2', 'robot_description_semantic.srdf')
    kinematics_yaml = os.path.join(pkg_share, 'config', 'moveit2', 'kinematics.yaml')
    controllers_yaml = os.path.join(pkg_share, 'config', 'moveit2', 'moveit_controllers.yaml')
    world_path = os.path.join(pkg_share, 'config', 'world', 'factory.world')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()
    with open(semantic_path, 'r', encoding='utf-8') as f:
        robot_description_semantic = f.read()

    # Gazebo
    ign_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '--verbose'],
        output='screen'
    )

    # ROS-GZ 桥接
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock]gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage]gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan]gz.msgs.LaserScan',
            '/camera/rgbd_camera/image_raw@sensor_msgs/msg/Image]gz.msgs.Image',
            '/camera/rgbd_camera/depth/image_raw@sensor_msgs/msg/Image]gz.msgs.Image',
            '/camera/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo]gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # 静态 TF
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 机器人状态发布
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        output='screen'
    )

    # 生成机器人
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-topic', 'robot_description',
             '-entity', 'robot',
             '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Nav2
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': os.path.join(pkg_share, 'config', 'nav2', 'map.yaml'),
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_share, 'config', 'nav2', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )

    # Rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ====== MoveGroup ======
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time},
            kinematics_yaml,
            controllers_yaml,
            {'request_adapters': [
                'default_planner_request_adapters/AddTimeParameterization',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ]},
        ],
    )

    # 控制器加载
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 视觉节点
    vision_control_node = Node(
        package="vision_perception",
        executable="vision_control_node.py",
        name="vision_control_node",
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            {"model_path": os.path.join(get_package_share_directory("vision_perception"), "weights", "best.onnx")},
            {"conf_threshold": 0.5},
            {"iou_threshold": 0.5},
            {"enable_dummy_detection": True},   # 暂时保持 dummy，确保流程跑通
        ]
    )

    # 导航节点（真实版本）
    navigation_node = Node(
        package='industry_manufacturing_line',
        executable='navigation_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 抓取节点（真实版本）
    manipulation_node = Node(
        package='industry_manufacturing_line',
        executable='manipulation_node',
        name='manipulation_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description_semantic': robot_description_semantic},
            kinematics_yaml,
        ]
    )

    # 主控制器
    robot_controller_node = Node(
        package='industry_manufacturing_line',
        executable='robot_controller_node',
        name='robot_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ign_sim,
        gz_bridge,
        static_map_to_odom,
        static_odom_to_base,
        nav2_bringup,
        robot_state_publisher,
        spawn_robot,
        rviz2,
        move_group,
        spawn_arm_controller,
        spawn_joint_state_broadcaster,
        vision_control_node,
        navigation_node,
        manipulation_node,
        robot_controller_node,
    ])