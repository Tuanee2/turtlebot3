#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher() : Node("initial_pose_publisher")
    {
        // Publisher for initialpose
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        // Subscriber for clicked_point
        subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10,
            std::bind(&InitialPosePublisher::handleClickedPoint, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Initial Pose Publisher has started.");
    }

private:
    void handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose.header.stamp = this->get_clock()->now();
        initial_pose.header.frame_id = "map";

        // Set the initial pose based on the clicked point
        initial_pose.pose.pose.position.x = msg->point.x;
        initial_pose.pose.pose.position.y = msg->point.y;
        initial_pose.pose.pose.orientation.w = 1.0;  // Neutral orientation

        // Publish the initial pose
        publisher_->publish(initial_pose);
        RCLCPP_INFO(this->get_logger(), "Published initial pose at x: %f, y: %f", msg->point.x, msg->point.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
