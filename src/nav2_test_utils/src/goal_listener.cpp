#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

class NavigateToPoseListener : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateToPoseListener()
        : Node("navigate_to_pose_listener")
    {
        RCLCPP_INFO(this->get_logger(), "start Listen NavigateToPose...");

        // 创建一个 Action 客户端，连接到 navigate_to_pose 服务
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        // 等待服务器连接
        while (!client_ptr_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to come up...");
        }

        // 打印导航目标
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavigateToPoseListener::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&NavigateToPoseListener::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&NavigateToPoseListener::result_callback, this, _1);

        // 定义一个空目标请求
        // 定义一个目标请求
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map"; // 确保使用正确的坐标系
        goal_msg.pose.pose.position.x = 0.0;
        goal_msg.pose.pose.position.y = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        // 发送目标请求
        auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);

        // 打印发送状态
        RCLCPP_INFO(this->get_logger(), "Goal sent to the server.");
    }

private:
    // 响应目标的回调函数
    void goal_response_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result...");
            RCLCPP_INFO(this->get_logger(), "timestamp: %f", goal_handle->get_goal_stamp().seconds());
        }
    }

    // 接收到反馈时的回调函数
    void
    feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // 打印反馈中的当前位置
        RCLCPP_INFO(this->get_logger(), "Current position: x = %.2f, y = %.2f",
                    feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
    }

    // 目标结果的回调函数
    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToPoseListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
