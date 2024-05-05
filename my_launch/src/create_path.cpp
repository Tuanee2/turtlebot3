#include <memory>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher") {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
        subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10,
            std::bind(&PathPublisher::handleClickedPoint, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Path Publisher has started. Create maximum 30 points for the path.");
    }

    void handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";
        if (path.poses.size() < 30) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position = msg->point;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
            RCLCPP_INFO(this->get_logger(), "Point added to path at x: %f, y: %f", msg->point.x, msg->point.y);
            //publisher_->publish(path);
        } else {
            std::cout << "Maximum number of points reached. Please send the path!" << std::endl;
        }
    }

    void publishPath() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!path.poses.empty()) {
            publisher_->publish(path);
            RCLCPP_INFO(this->get_logger(), "Final path published with %zu points.", path.poses.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "No path to send.");
        }
    }

private:
    nav_msgs::msg::Path path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
    std::mutex mutex_;
};

void userInteractionThread(std::shared_ptr<PathPublisher> node) {
    std::cout << "Press 's' or 'S' to send the path and terminate" << std::endl;
    char input;
    while (std::cin >> input) {
        if (input == 's' || input == 'S') {
            node->publishPath();
            rclcpp::shutdown();
            break;
        } else {
            std::cout << "Invalid command! Please press 'S' or 's' to send the path and terminate" << std::endl;
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    std::thread userInputThread(userInteractionThread, node);
    rclcpp::spin(node);
    userInputThread.join();
    return 0;
}
