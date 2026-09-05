import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('industry_manufacturing_line')
    world_path = os.path.join(pkg_share, 'config', 'world', 'factory.world')
    urdf_path = os.path.join(pkg_share, 'config', 'robot', 'robot_description.urdf')
    rtabmap_config_path = os.path.join(pkg_share, 'config', 'nav2', 'rtabmap.ini')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    ign_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '--verbose'],
        output='screen'
    )

    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock]gz.msgs.Clock',
        '/camera/rgbd_camera/image_raw@sensor_msgs/msg/Image]gz.msgs.Image',
        '/camera/rgbd_camera/depth/image_raw@sensor_msgs/msg/Image]gz.msgs.Image',
        '/camera/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo]gz.msgs.CameraInfo',
        '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan]gz.msgs.LaserScan',
    ]
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_topics,
        output="screen"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description}
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'approx_sync': True},
            {'approx_sync_max_interval': 0.5},
            {'topic_queue_size': 100},
            {'sync_queue_size': 100},
            {'odom_sensor_sync': True},
        ],
        remappings=[
            ('rgb/image', '/camera/rgbd_camera/image_raw'),
            ('depth/image', '/camera/rgbd_camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/rgbd_camera/camera_info'),
            ('rgbd_image', 'rgbd_image')
        ]
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_path': rtabmap_config_path},
            {'database_path': '/tmp/rtabmap_new.db'},
            {'map_grid': True},
            {'map_2d': True},
            {'publish_tf': True},
            {'publish_tf_map': True},
            {'map_frame_id': 'map'},
            {'odom_frame_id': 'odom'},
            {'frame_id': 'base_link'},
            {'rgbd_cameras': 1},
            {'subscribe_rgbd': True},
            {'subscribe_rgb': False},
            {'subscribe_depth': False},
            {'wait_for_transform': 2.0},
            {'approx_sync': True},
            {'sync_queue_size': 30},
            {'topic_queue_size': 10},
            {'subscribe_scan': True},
            {'scan_topic': '/scan'},
            {'scan_frame_id': 'base_scan'},
        ],
        remappings=[
            ('rgbd_image', 'rgbd_image'),
            ('odom', '/odom'),
            ('scan', '/scan'),
        ]
    )

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_scan_broadcaster',
        arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'base_scan'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ign_sim,
        gz_bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        rgbd_sync,
        rtabmap,
        static_transform_publisher,
        rviz2
    ])