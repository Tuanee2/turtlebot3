#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <vector>

class WorkAreaMonitor : public rclcpp::Node
{
public:
    WorkAreaMonitor() : Node("work_area_monitor")
    {
        // Đăng ký để nhận vị trí của robot từ AMCL
        position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&WorkAreaMonitor::position_callback, this, std::placeholders::_1));
        
        // Đăng ký để nhận vùng làm việc
        work_area_subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "work_area", 10, std::bind(&WorkAreaMonitor::work_area_callback, this, std::placeholders::_1));

        // Publisher để gửi lệnh dừng robot
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void position_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_position_.point.x = msg->pose.pose.position.x;
        current_position_.point.y = msg->pose.pose.position.y;
        current_position_.point.z = msg->pose.pose.position.z;
        check_and_stop_robot();
    }

    void work_area_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {
        work_area_ = msg->polygon;
    }

    void check_and_stop_robot()
    {
        if (!is_point_in_polygon(current_position_.point, work_area_)) {
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.linear.y = 0.0;
            stop_msg.linear.z = 0.0;
            stop_msg.angular.x = 0.0;
            stop_msg.angular.y = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(stop_msg);
            RCLCPP_INFO(this->get_logger(), "Robot is outside the work area. Sending stop command.");
        }
    }

    bool is_point_in_polygon(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::Polygon& polygon)
    {
        int i, j, nvert = polygon.points.size();
        bool inside = false;

        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            if (((polygon.points[i].y >= point.y) != (polygon.points[j].y >= point.y)) &&
                (point.x <= (polygon.points[j].x - polygon.points[i].x) * (point.y - polygon.points[i].y) / (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x))
                inside = !inside;
        }

        return inside;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr position_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr work_area_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    geometry_msgs::msg::PointStamped current_position_;
    geometry_msgs::msg::Polygon work_area_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorkAreaMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
