#include <rclcpp/rclcpp.hpp>
#include "industry_manufacturing_line/srv/grasp_object.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class ManipulationNode : public rclcpp::Node
{
public:
    ManipulationNode() : Node("manipulation_node")
    {
        this->declare_parameter<std::string>("planning_group", "arm_group");
        this->declare_parameter<std::string>("end_effector_link", "gripper_link");
        planning_group_ = this->get_parameter("planning_group").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();

        grasp_service_ = this->create_service<industry_manufacturing_line::srv::GraspObject>(
            "/grasp_object",
            std::bind(&ManipulationNode::handle_grasp, this,
                std::placeholders::_1, std::placeholders::_2
            )
        );

        RCLCPP_INFO(this->get_logger(),
            "Manipulation Node Ready. Group: [%s], End‑effector: [%s]",
            planning_group_.c_str(), end_effector_link_.c_str());
    }

private:
    // 修正：函数参数列表干净，逻辑全部放到{}内部
    void handle_grasp(
        const std::shared_ptr<industry_manufacturing_line::srv::GraspObject::Request> request,
        std::shared_ptr<industry_manufacturing_line::srv::GraspObject::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received grasp request for: %s", request->object_id.c_str());
        try {
            rclcpp::Node::SharedPtr node_ptr = this->shared_from_this();
            moveit::planning_interface::MoveGroupInterface move_group(node_ptr, planning_group_);

            // ===== 强制使用 OMPL 规划器（放到函数体内） =====
            move_group.setPlanningPipelineId("ompl");
            move_group.setPlannerId("RRTConnect");
            // =========================================

            move_group.setMaxVelocityScalingFactor(0.3);
            move_group.setMaxAccelerationScalingFactor(0.3);
            move_group.setPlanningTime(2.0);

            geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link_);
            geometry_msgs::msg::Pose target_pose;

            if (request->target_pose.position.x != 0.0 || request->target_pose.position.y != 0.0) {
                target_pose = request->target_pose;
                RCLCPP_INFO(this->get_logger(), "使用视觉坐标: x=%.3f, y=%.3f",
                            target_pose.position.x, target_pose.position.y);
            } else {
                target_pose = current_pose.pose;
                target_pose.position.x += 0.05;
                RCLCPP_INFO(this->get_logger(), "使用默认偏移坐标");
            }

            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                RCLCPP_INFO(this->get_logger(), "✅ 抓取规划成功，开始执行轨迹");
                auto exec_result = move_group.execute(plan);
                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "✅ 轨迹执行完成");
                    response->success = true;
                    response->message = "Grasp executed successfully for " + request->object_id;
                } else {
                    RCLCPP_WARN(this->get_logger(), "❌ 轨迹执行失败");
                    response->success = false;
                    response->message = "Trajectory execution failed.";
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "❌ 规划失败，尝试使用关节目标作为备选方案");
                std::vector<double> joint_goal = {0.5, 0.3, 0.1};
                move_group.setJointValueTarget(joint_goal);
                success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "✅ 关节目标规划成功，开始执行");
                    auto exec_result = move_group.execute(plan);
                    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "✅ 关节轨迹执行完成");
                        response->success = true;
                        response->message = "Grasp executed (joint goal) for " + request->object_id;
                    } else {
                        response->success = false;
                        response->message = "Joint trajectory execution failed.";
                    }
                } else {
                    response->success = false;
                    response->message = "Both pose and joint planning failed.";
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt error: %s", e.what());
            response->success = false;
            response->message = "Exception: " + std::string(e.what());
        }
    }

    rclcpp::Service<industry_manufacturing_line::srv::GraspObject>::SharedPtr grasp_service_;
    std::string planning_group_;
    std::string end_effector_link_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
