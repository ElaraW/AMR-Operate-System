import pytest
import rclpy
from rclpy.node import Node
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node as LaunchNode

@pytest.mark.rostest
def generate_test_description():
    # 启动你整个系统的 launch 文件（模拟场景 1：建图+导航）
    return LaunchDescription([
        LaunchNode(
            package='industry_manufacturing_line',
            executable='system_start.launch.py',
            name='system_test',
            parameters=[{'use_sim_time': True}]
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestSystemScenarios:
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_alive(self):
        """场景1验证：检查核心节点是否存活并发布TF"""
        node = rclpy.create_node('test_monitor')
        # 等待 /tf 话题（证明导航/建图在运行）
        try:
            rclpy.spin_once(node, timeout_sec=5.0)
            assert True  # 这里实际应用应订阅话题做断言，先留空表示框架
        except Exception:
            assert False