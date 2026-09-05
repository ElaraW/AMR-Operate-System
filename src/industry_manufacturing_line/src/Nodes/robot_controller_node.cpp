#include <rclcpp/rclcpp.hpp>
#include "industry_manufacturing_line/srv/navigate_to_station.hpp"
#include "industry_manufacturing_line/srv/grasp_object.hpp"
#include "industry_manufacturing_line/msg/detection_object.hpp"
#include <memory>
#include <functional>

using namespace std::chrono_literals;

enum class RobotState {
    IDLE,
    NAVIGATING,
    PERCEPTION_WAIT,
    GRASPING,
    PLACING,
    RETURNING,
    COMPLETE
};

class RobotControllerNode : public rclcpp::Node
{
public:
    RobotControllerNode() : Node("robot_controller_node"),
                            state_(RobotState::IDLE),
                            cycle_count_(0)
    {
        nav_client_ = this->create_client<industry_manufacturing_line::srv::NavigateToStation>(
            "/navigate_to_station");
        grasp_client_ = this->create_client<industry_manufacturing_line::srv::GraspObject>(
            "/grasp_object");

        detection_sub_ = this->create_subscription<industry_manufacturing_line::msg::DetectionObject>(
            "/detection_objects", 10,
            std::bind(&RobotControllerNode::detection_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(100ms, std::bind(&RobotControllerNode::state_machine, this));

        RCLCPP_INFO(this->get_logger(), "=== Robot Controller State Machine Started ===");
    }

private:
    void state_machine()
    {
        switch (state_)
        {
        case RobotState::IDLE:
            RCLCPP_INFO(this->get_logger(), "[STATE] IDLE -> Start Navigation to Station A (Cycle %d)", cycle_count_ + 1);
            nav_done_ = false;
            grasp_done_ = false;
            perception_received_ = false;
            nav_timeout_counter_ = 0;
            grasp_timeout_counter_ = 0;

            send_nav_request("Station_A");
            state_ = RobotState::NAVIGATING;
            break;

        case RobotState::NAVIGATING:
            if (nav_done_) {
                RCLCPP_INFO(this->get_logger(), "[STATE] NAVIGATING -> Wait for Perception");
                nav_done_ = false;
                nav_timeout_counter_ = 0;
                state_ = RobotState::PERCEPTION_WAIT;
            } else {
                nav_timeout_counter_++;
                if (nav_timeout_counter_ > 300) {
                    RCLCPP_WARN(this->get_logger(), "[STATE] NAVIGATING timeout! Force proceed.");
                    nav_done_ = true;
                    nav_timeout_counter_ = 0;
                }
            }
            break;

        case RobotState::PERCEPTION_WAIT:
            if (perception_received_) {
                RCLCPP_INFO(this->get_logger(), "[STATE] PERCEPTION -> Grasping Object");
                perception_received_ = false;
                send_grasp_request("detected_box");
                state_ = RobotState::GRASPING;
            } else {
                static int wait_count = 0;
                wait_count++;
                if (wait_count > 100) {
                    RCLCPP_WARN(this->get_logger(), "[STATE] PERCEPTION timeout! Sending dummy grasp.");
                    wait_count = 0;
                    send_grasp_request("dummy_box");
                    state_ = RobotState::GRASPING;
                }
            }
            break;

        case RobotState::GRASPING:
            if (grasp_done_) {
                RCLCPP_INFO(this->get_logger(), "[STATE] GRASPING -> Placing to Station B");
                grasp_done_ = false;
                grasp_timeout_counter_ = 0;
                send_nav_request("Station_B");
                state_ = RobotState::PLACING;
            } else {
                grasp_timeout_counter_++;
                if (grasp_timeout_counter_ > 100) {
                    RCLCPP_WARN(this->get_logger(), "[STATE] GRASPING timeout! Force proceed.");
                    grasp_done_ = true;
                    grasp_timeout_counter_ = 0;
                }
            }
            break;

        case RobotState::PLACING:
            if (nav_done_) {
                RCLCPP_INFO(this->get_logger(), "[STATE] PLACING -> Place executed, Returning Home");
                nav_done_ = false;
                nav_timeout_counter_ = 0;
                send_nav_request("Home");
                state_ = RobotState::RETURNING;
            } else {
                nav_timeout_counter_++;
                if (nav_timeout_counter_ > 300) {
                    RCLCPP_WARN(this->get_logger(), "[STATE] PLACING timeout! Force return.");
                    nav_done_ = true;
                    nav_timeout_counter_ = 0;
                }
            }
            break;

        case RobotState::RETURNING:
            if (nav_done_) {
                RCLCPP_INFO(this->get_logger(), "[STATE] RETURNING -> Cycle %d Complete!", cycle_count_ + 1);
                nav_done_ = false;
                cycle_count_++;
                state_ = RobotState::IDLE;

                if (cycle_count_ >= 5) {
                    RCLCPP_INFO(this->get_logger(), "✅ 已完成5个工作循环，演示结束。");
                    rclcpp::shutdown();
                }
            } else {
                nav_timeout_counter_++;
                if (nav_timeout_counter_ > 300) {
                    RCLCPP_WARN(this->get_logger(), "[STATE] RETURNING timeout! Force back to IDLE.");
                    nav_done_ = true;
                    nav_timeout_counter_ = 0;
                }
            }
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Unknown state!");
            state_ = RobotState::IDLE;
            break;
        }
    }

    void send_nav_request(const std::string &station)
    {
        if (!nav_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Navigation service not available! Simulating success.");
            nav_done_ = true;
            return;
        }
        auto req = std::make_shared<industry_manufacturing_line::srv::NavigateToStation::Request>();
        req->station_name = station;
        nav_client_->async_send_request(req,
            std::bind(&RobotControllerNode::nav_response_callback, this, std::placeholders::_1));
        nav_done_ = false;
    }

    void nav_response_callback(
        rclcpp::Client<industry_manufacturing_line::srv::NavigateToStation>::SharedFuture future)
    {
        auto res = future.get();
        if (res->success) {
            RCLCPP_INFO(this->get_logger(), "✅ Navigation succeeded : %s", res->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ Navigation failed : %s", res->message.c_str());
        }
        nav_done_ = true;
    }

    // 发送抓取请求时携带视觉坐标
    void send_grasp_request(const std::string &obj)
    {
        if (!grasp_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Grasp service not available! Simulating success.");
            grasp_done_ = true;
            return;
        }
        auto req = std::make_shared<industry_manufacturing_line::srv::GraspObject::Request>();
        req->object_id = obj;
        // 若视觉检测到了目标，把坐标传给抓取服务
        if (perception_received_ && has_target_pose_) {
            req->target_pose = target_pose_;
            RCLCPP_INFO(this->get_logger(), " 使用视觉坐标抓取: x=%.2f, y=%.2f",
                        target_pose_.position.x, target_pose_.position.y);
        } else {
            // 没有视觉坐标时使用默认让 manipulation_node 自己用固定偏移
            RCLCPP_WARN(this->get_logger(), "⚠️ 没有视觉坐标，使用默认抓取位置");
        }
        grasp_client_->async_send_request(req,
            std::bind(&RobotControllerNode::grasp_response_callback, this, std::placeholders::_1));
        grasp_done_ = false;
    }

    void grasp_response_callback(
        rclcpp::Client<industry_manufacturing_line::srv::GraspObject>::SharedFuture future)
    {
        auto res = future.get();
        if (res->success) {
            RCLCPP_INFO(this->get_logger(), "✅ Grasp succeeded: %s", res->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ Grasp failed: %s", res->message.c_str());
        }
        grasp_done_ = true;
    }

    // 接收 DetectionObject
    void detection_callback(const industry_manufacturing_line::msg::DetectionObject::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), " 收到视觉检测: color=%s, april_id=%d, conf=%.2f",
                    msg->color.c_str(), msg->april_id, msg->confidence);

        // 保存视觉坐标，供抓取服务使用
        target_pose_ = msg->pose;
        has_target_pose_ = true;

        // 触发状态机前进
        perception_received_ = true;
    }

    // ---- 成员变量 ----
    RobotState state_;
    rclcpp::TimerBase::SharedPtr timer_;
    int cycle_count_;

    rclcpp::Client<industry_manufacturing_line::srv::NavigateToStation>::SharedPtr nav_client_;
    rclcpp::Client<industry_manufacturing_line::srv::GraspObject>::SharedPtr grasp_client_;
    rclcpp::Subscription<industry_manufacturing_line::msg::DetectionObject>::SharedPtr detection_sub_;

    bool nav_done_ = false;
    bool grasp_done_ = false;
    bool perception_received_ = false;

    int nav_timeout_counter_ = 0;
    int grasp_timeout_counter_ = 0;

    // 存储视觉检测到的目标位姿
    geometry_msgs::msg::Pose target_pose_;
    bool has_target_pose_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}