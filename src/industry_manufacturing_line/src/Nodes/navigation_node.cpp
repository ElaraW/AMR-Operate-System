#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "industry_manufacturing_line/srv/navigate_to_station.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <map>
#include <string>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <mutex>

using namespace std::chrono_literals;

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node")
    {
        station_poses_["Station_A"] = createPose(2.0, 1.0, 0.0);
        station_poses_["Station_B"] = createPose(-1.5, -0.5, 0.0);
        station_poses_["Home"] = createPose(0.0, 0.0, 0.0);

        service_ = this->create_service<industry_manufacturing_line::srv::NavigateToStation>(
            "/navigate_to_station",
            std::bind(&NavigationNode::handle_navigate, this, std::placeholders::_1, std::placeholders::_2)
        );

        nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Navigation Node ready with Nav2 real action client.");
    }

private:
    geometry_msgs::msg::Pose createPose(double x, double y, double yaw)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.orientation = tf2::toMsg(q);
        return pose;
    }

    void handle_navigate(
        const std::shared_ptr<industry_manufacturing_line::srv::NavigateToStation::Request> request,
        std::shared_ptr<industry_manufacturing_line::srv::NavigateToStation::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Navigate to station: %s (real Nav2)", request->station_name.c_str());

        auto it = station_poses_.find(request->station_name);
        if (it == station_poses_.end()) {
            response->success = false;
            response->message = "Unknown station: " + request->station_name;
            return;
        }

        if (!nav_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
            response->success = false;
            response->message = "Nav2 action server not available";
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose = it->second;

        // 使用共享指针保证生命周期
        auto result = std::make_shared<std::pair<bool, bool>>(false, false);
        std::mutex mtx;
        std::condition_variable cv;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [&result, &cv, &mtx](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & handle) {
                std::lock_guard<std::mutex> lock(mtx);
                result->first = true;  // goal accepted
                result->second = (handle != nullptr);
                cv.notify_one();
            };
        send_goal_options.result_callback =
            [&result, &cv, &mtx](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & res) {
                std::lock_guard<std::mutex> lock(mtx);
                result->first = false;  // done
                result->second = (res.code == rclcpp_action::ResultCode::SUCCEEDED);
                cv.notify_one();
            };

        nav_action_client_->async_send_goal(goal_msg, send_goal_options);

        // 等待结果（最多30秒）
        std::unique_lock<std::mutex> lock(mtx);
        auto status = cv.wait_for(lock, 30s, [&result]() { return !result->first; });

        if (status && result->second) {
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            response->success = true;
            response->message = "Navigation to " + request->station_name + " succeeded";
        } else if (status && !result->second) {
            RCLCPP_WARN(this->get_logger(), "Navigation failed");
            response->success = false;
            response->message = "Navigation failed";
        } else {
            RCLCPP_WARN(this->get_logger(), "Navigation timed out");
            response->success = false;
            response->message = "Navigation timed out";
        }
    }

    rclcpp::Service<industry_manufacturing_line::srv::NavigateToStation>::SharedPtr service_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
    std::map<std::string, geometry_msgs::msg::Pose> station_poses_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}