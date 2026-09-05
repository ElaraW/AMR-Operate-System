import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前包路径 
    pkg_path = get_package_share_directory('industry_manufacturing_line')

    # URDF 文件路径
    urdf_path = os.path.join(pkg_path, 'config', 'robot', 'robot_description.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # MoveIt2 配置文件路径
    kinematics_yaml = os.path.join(pkg_path, 'config', 'moveit2', 'kinematics.yaml')
    joint_limits_yaml = os.path.join(pkg_path, 'config', 'moveit2', 'joint_limits.yaml')
    ompl_yaml = os.path.join(pkg_path, 'config', 'moveit2', 'ompl_planning.yaml')

    # 核心：move_group 节点
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True},
            kinematics_yaml,
            joint_limits_yaml,
            ompl_yaml,
            # 下面这些是自动生成的 robot_description 参数（固定写法）
            {'robot_description_semantic': ''},
            {'robot_description_kinematics': ''},
            {'robot_description_planning': ''}
        ],
    )

    # Rviz2 可视化节点（便于查看规划结果）
    rviz_config_path = os.path.join(pkg_path, 'config', 'moveit2', 'moveit.rviz')
    # 如果你还没有 moveit.rviz 文件，可以把下面这行注释掉，手动开 Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
    )

    # 静态 TF 发布（固定 base_footprint 到 odom，保持 TF 树完整）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    return LaunchDescription([
        static_tf_node,
        move_group_node,
        rviz_node,
    ])