### 项目简介

本项目面向工业移动机器人研发场景，构建了一个模块化、可扩展的仿真测试底座。框架基于 ROS2 Humble 设计，独立编写 URDF 机器人模型，配置 Gazebo 物理仿真环境、差速运动控制器与多传感器仿真插件。

以 RTAB‑Map、YOLOv8、Nav2、MoveIt2 为示例模块，统一数据交互接口，通过状态机实现多任务流程调度。配套自动化测试脚本覆盖 5 类典型工业作业场景，支持持续集成与回归验证。

# 环境要求
依赖	       版本
Ubuntu	    2.04 LTS
ROS2	      Humble
Gazebo	    Classic (Fortress)
Python	    3.10+
CMake	      3.22+


# 技术栈

机器人框架：ROS2 Humble

仿真环境：Gazebo Fortress

机器人建模：URDF

感知算法：YOLOv8 (ONNX Runtime), RTAB-Map, OpenCV

导航规划：Nav2

运动规划：MoveIt2 + OMPL

编程语言：C++  Python

版本管理：Git

# 系统架构

仿真层：物理仿真，URDF机器人模型，多传感器仿真插件

感知层：YOLOv8目标检测，RTAB-Map融合建图

规划控制层：Nav2导航规划，MoveIt2运动控制

测试层：Rviz2可视化，自动化测试脚本


# 5 类作业场景

场景 1：静态地图导航  Nav2 全局路径规划 + 纯定位

场景 2：动态障碍物避障  Nav2 局部规划器响应移动障碍

场景 3：视觉引导识别与定位  YOLO 检测 + AprilTag 定位 + TF 坐标转换

场景 4：机械臂抓取与放置  MoveIt2 运动规划 + 抓取执行

场景 5：多任务连续执行  状态机串联：导航→识别→抓取→放置→复位循环


### 安装依赖

# 安装 ROS2 基础包
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz-bridge ros-humble-gz-ros2-control

# 安装 Nav2 和 MoveIt2
sudo apt install ros-humble-nav2-bringup ros-humble-moveit ros-humble-moveit-planners-ompl

# 安装 Python 依赖
pip3 install onnxruntime opencv-python numpy ultralytics



### 克隆与编译

# 创建工作空间
mkdir -p ~/AMR-Operate-System/src
cd ~/AMR-Operate-System/src

# 克隆仓库
git clone https://github.com/

# 返回工作空间根目录并编译
colcon build --symlink-install
source install/setup.bash

# 运行仿真
ros2 launch industry_manufacturing_line system_start.launch.py


### 项目结构
```
AMR-Operate-System
├── industry_manufacturing_line/
│   ├── config/
│   │   ├── moveit2/
│   │   │   ├── arm_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── moveit_controllers.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── robot_description_semantic.srdf
│   │   ├── nav2/
│   │   │   ├── map.pgm
│   │   │   ├── map.yaml
│   │   │   ├── nav2_params.yaml
│   │   │   └── rtabmap.ini
│   │   ├── perception/
│   │   │   ├── apriltag_config.yaml
│   │   │   └── yolo_config.yaml
│   │   ├── robot/
│   │   │   └── robot_description.urdf
│   │   └── world/
│   │       └── factory.world
│   ├── launch/
│   │   ├── mapping.launch.py
│   │   ├── moveit.launch.py
│   │   ├── navigation.launch.py
│   │   └── system_start.launch.py
│   ├── msg/
│   │   ├── DetectionObject.msg
│   │   └── TaskFeedback.msg
│   ├── src/
│   │   ├── Nodes/
│   │   │   ├── manipulation_node.cpp
│   │   │   ├── navigation_node.cpp
│   │   │   └── robot_controller_node.cpp
│   │   └── main.cpp
│   ├── srv/
│   │   ├── GraspObject.srv
│   │   └── NavigateToStation.srv
│   ├── test/
│   │   └── test_system_bringup.py
│   ├── CMakeLists.txt
│   └── package.xml
└── vision_perception/
├── launch/
│   └── vision_control.launch.py
├── resource/
│   └── vision_perception
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── vision_perception/
│   ├── **init**.py
│   └── vision_control_node.py
├── weights/
│   └── best.onnx
├── package.xml
└── setup.py
```

# License
本项目采用 MIT License 开源