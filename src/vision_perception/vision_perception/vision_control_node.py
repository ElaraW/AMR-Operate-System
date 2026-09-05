#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist, PoseStamped, Pose
from cv_bridge import CvBridge
import cv2
import math
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from industry_manufacturing_line.msg import DetectionObject

torch = None
YOLO = None

class VisionControlNode(Node):
    def __init__(self):
        super().__init__('vision_control_node')

        # 参数声明：全部只声明一次！！
        self.declare_parameter('enable_dummy_detection', False)
        self.declare_parameter('model_path', 'best.onnx')
        self.declare_parameter('conf_threshold', 0.15)
        self.declare_parameter('iou_threshold', 0.35)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('control_rate', 10.0)

        # 获取参数
        self.enable_dummy = self.get_parameter('enable_dummy_detection').value
        self.model_path = self.get_parameter('model_path').value
        self.conf_thres = self.get_parameter('conf_threshold').value
        self.iou_thres = self.get_parameter('iou_threshold').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.rate_hz = self.get_parameter('control_rate').value

        self.model = None
        # 只有非dummy模式，才导入torch、YOLO并且加载模型
        if not self.enable_dummy:
            global torch, YOLO
            os.environ["CUDA_VISIBLE_DEVICES"] = ""
            import torch
            torch.cuda.is_available = lambda: False
            from ultralytics import YOLO

            if not os.path.exists(self.model_path):
                self.get_logger().error(f"模型文件不存在: {self.model_path}")
                self.model = None
            else:
                self.model = YOLO(self.model_path, task="detect")
                self.get_logger().info(f"成功加载YOLO模型: {self.model_path}")

        self.bridge = CvBridge()

        # 3. ROS订阅/发布接口
        self.img_sub = self.create_subscription(
            Image, '/camera/rgbd_camera/image_raw', self.image_cb, 10
        )
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray, '/tag_detections', self.tag_cb, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.target_pub = self.create_publisher(String, "/fusion_target_info", 20)
        self.target_pose_pub = self.create_publisher(Pose, "/fusion_target_pose", 20)
        self.detection_pub = self.create_publisher(DetectionObject, '/detection_objects', 10)

        # 4. TF 缓存 + 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. 缓存最新数据
        self.latest_cv_img = None
        self.latest_tags = []
        self.last_base_pose = None

        # 6. 定时控制循环
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)
        self.get_logger().info("YOLO+AprilTag 视觉节点启动完成")

        # 7. 模拟检测（dummy模式）
        if self.enable_dummy:
            self.dummy_timer = self.create_timer(2.0, self.publish_dummy_detection)
            self.get_logger().warn("模拟检测已启用，将每2秒发送假目标")

    def image_cb(self, msg):
        try:
            self.latest_cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {str(e)}")
            self.latest_cv_img = None

    def tag_cb(self, msg):
        self.latest_tags = msg.detections

    def yolo_infer(self, img):
        if img is None or self.model is None:
            return []
        results = self.model(img, conf=self.conf_thres, iou=self.iou_thres, verbose=False)
        detect_list = []
        for res in results:
            if res.boxes is None:
                continue
            for box in res.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                detect_list.append({
                    "cls": cls_name,
                    "conf": conf,
                    "center_x": cx,
                    "center_y": cy,
                    "bbox": (x1, y1, x2, y2)
                })
        return detect_list

    def match_tag_yolo(self, tag_list, yolo_list, pixel_thresh=80):
        if not tag_list or not yolo_list:
            return None
        for tag in tag_list:
            if not hasattr(tag, "center") or tag.center is None:
                continue
            tag_cx = tag.center.x
            tag_cy = tag.center.y
            best_match = None
            min_dist = float("inf")
            for yolo_obj in yolo_list:
                dist = math.hypot(yolo_obj["center_x"] - tag_cx, yolo_obj["center_y"] - tag_cy)
                if dist < min_dist and dist < pixel_thresh:
                    min_dist = dist
                    best_match = yolo_obj
            if best_match is not None:
                return tag, best_match
        return None

    def control_loop(self):
        if self.latest_cv_img is None:
            return
        yolo_dets = self.yolo_infer(self.latest_cv_img)
        match_result = self.match_tag_yolo(self.latest_tags, yolo_dets)
        if match_result is not None:
            tag_info, color_obj = match_result
            color_name = color_obj["cls"]
            tag_id = tag_info.id[0]
            self.get_logger().info(f"匹配成功：色块={color_name}, TagID={tag_id}")

            info_msg = String()
            info_msg.data = f"tag_id:{tag_id}, object_class:{color_name}"
            self.target_pub.publish(info_msg)

            tag_pose = tag_info.pose.pose.pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "camera_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = tag_pose
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    "camera_link",
                    rclpy.time.Time()
                )
                base_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                x = base_pose.pose.position.x
                y = base_pose.pose.position.y
                self.last_base_pose = base_pose.pose

                pose_msg = Pose()
                pose_msg.position.x = x
                pose_msg.position.y = y
                pose_msg.position.z = base_pose.pose.position.z
                pose_msg.orientation = base_pose.pose.orientation
                self.target_pose_pub.publish(pose_msg)

                detection_msg = DetectionObject()
                detection_msg.color = color_name
                detection_msg.april_id = tag_id
                detection_msg.pose = base_pose.pose
                detection_msg.confidence = color_obj["conf"]
                self.detection_pub.publish(detection_msg)

                self.get_logger().info(
                    f"📤 已发布 DetectionObject: color={color_name}, id={tag_id}, pos=({x:.2f}, {y:.2f})"
                )
                target_yaw = math.atan2(y, x)
                twist = Twist()
                if abs(target_yaw) > self.angle_tol:
                    twist.angular.z = self.angular_gain * target_yaw
                    twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)
                    self.cmd_pub.publish(twist)
                else:
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
            except Exception as e:
                self.get_logger().warn(f"TF坐标变换失败: {str(e)}")

    def publish_dummy_detection(self):
        """模拟检测发布（用于无真实传感器时的测试）"""
        msg = DetectionObject()
        msg.color = "dummy_red_box"
        msg.april_id = 1
        dummy_pose = Pose()
        dummy_pose.position.x = 0.5
        dummy_pose.position.y = 0.2
        dummy_pose.position.z = 0.1
        dummy_pose.orientation.w = 1.0
        msg.pose = dummy_pose
        msg.confidence = 0.95
        self.detection_pub.publish(msg)
        self.get_logger().info("📤 已发布模拟 DetectionObject (dummy)")

    def stop_car(self):
        stop_msg = Twist()
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
