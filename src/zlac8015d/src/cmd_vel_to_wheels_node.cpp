#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "overlord100_msgs/msg/wheels_data.hpp"

class CmdVelToWheelsNode : public rclcpp::Node
{
public:
    CmdVelToWheelsNode() : Node("cmd_vel_to_wheels_node")
    {
        // 参数配置
        this->declare_parameter("wheel_base", 0.3);  // 轮距（米）
        this->declare_parameter("max_linear_speed", 2.0);  // 最大线速度（米/秒）
        this->declare_parameter("max_angular_speed", 2.0); // 最大角速度（弧度/秒）
        
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        
        // 创建订阅者和发布者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&CmdVelToWheelsNode::cmdVelCallback, this, std::placeholders::_1));
            
        wheels_control_pub_ = this->create_publisher<overlord100_msgs::msg::WheelsData>(
            "wheels_control", 10);
            
        RCLCPP_INFO(this->get_logger(), "CmdVel to Wheels 转换节点已启动");
        RCLCPP_INFO(this->get_logger(), "轮距: %.2f m, 最大线速度: %.2f m/s, 最大角速度: %.2f rad/s", 
                   wheel_base_, max_linear_speed_, max_angular_speed_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto wheels_cmd = std::make_unique<overlord100_msgs::msg::WheelsData>();
        
        // 限制速度范围
        double linear_x = std::max(-max_linear_speed_, std::min(max_linear_speed_, msg->linear.x));
        double angular_z = std::max(-max_angular_speed_, std::min(max_angular_speed_, msg->angular.z));
        
        // 差分驱动运动学模型
        // v_left = v_x - (omega * wheel_base / 2)
        // v_right = v_x + (omega * wheel_base / 2)
        
        wheels_cmd->left = linear_x - (angular_z * wheel_base_ / 2.0);
        wheels_cmd->right = linear_x + (angular_z * wheel_base_ / 2.0);
        
        // 轮速限制（假设轮速范围在 -max_linear_speed 到 max_linear_speed 之间）
        double max_wheel_speed = max_linear_speed_ * 1.2;  // 给轮速留一些余量
        wheels_cmd->left = std::max(-max_wheel_speed, std::min(max_wheel_speed, wheels_cmd->left));
        wheels_cmd->right = std::max(-max_wheel_speed, std::min(max_wheel_speed, wheels_cmd->right));
        
        wheels_control_pub_->publish(std::move(wheels_cmd));
        
        // 调试输出
        RCLCPP_DEBUG(this->get_logger(), 
                    "接收到 cmd_vel: linear=%.3f, angular=%.3f -> 轮速: left=%.3f, right=%.3f",
                    linear_x, angular_z, wheels_cmd->left, wheels_cmd->right);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<overlord100_msgs::msg::WheelsData>::SharedPtr wheels_control_pub_;
    
    double wheel_base_;
    double max_linear_speed_;
    double max_angular_speed_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToWheelsNode>());
    rclcpp::shutdown();
    return 0;
}