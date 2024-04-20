#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class AMCLPoseSubscriber : public rclcpp::Node
{
public:
    AMCLPoseSubscriber() : Node("amcl_pose_subscriber")
    {
        // Đăng ký vào topic "/amcl_pose" với hàm xử lý callback là processAMCLPoseMessage
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            10,
            std::bind(&AMCLPoseSubscriber::processAMCLPoseMessage, this, std::placeholders::_1));
    }

private:
    // Hàm callback để xử lý các tin nhắn nhận được từ topic "/amcl_pose"
    void processAMCLPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // In ra thông tin về tin nhắn nhận được
        RCLCPP_INFO(this->get_logger(), "Received AMCL Pose:\n"
                                         "Position:\n"
                                         "  x: %f\n"
                                         "  y: %f\n"
                                         "  z: %f\n"
                                         "Orientation:\n"
                                         "  x: %f\n"
                                         "  y: %f\n"
                                         "  z: %f\n"
                                         "  w: %f",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Tạo một instance của lớp AMCLPoseSubscriber
    auto node = std::make_shared<AMCLPoseSubscriber>();
    printf("wait the pose data");
    // Spin node để xử lý các tin nhắn
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
