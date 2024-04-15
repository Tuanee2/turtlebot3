#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DrawCircleNode : public rclcpp::Node {
public:
  DrawCircleNode() : Node("draw_circle") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&DrawCircleNode::send_velocity_command, this));
    RCLCPP_INFO(this->get_logger(), "draw circle node has been started");
  }

private:
  void send_velocity_command() {
    // Tạo std::unique_ptr chứa tin nhắn Twist
    auto msg_unique_ptr = std::make_unique<geometry_msgs::msg::Twist>();

    // Thiết lập các giá trị cho msg
    msg_unique_ptr->linear.x = 2.0;
    msg_unique_ptr->angular.z = 1.0;

    // Gửi tin nhắn
    cmd_vel_pub_->publish(std::move(msg_unique_ptr));
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrawCircleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
