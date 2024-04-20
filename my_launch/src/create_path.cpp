#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // Publisher for /plan
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
        // Subscriber for clicked_point
        subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10,
            std::bind(&PathPublisher::handleClickedPoint, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Path Publisher has started. Create maximum 30 point for tha path.");
    }

    ~PathPublisher()
    {
        publishPath();  // Publish the final path when node is shutdown
    }

private:
    void handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        //nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";
        if(path.poses.size()<=30){
            // Create a simple path based on the clicked point
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = msg->point.x;  // Directly use the clicked x
            pose.pose.position.y = msg->point.y;  // Directly use the clicked y
            pose.pose.position.z = 0.0;           // Assume flat surface
            pose.pose.orientation.w = 1.0;        // Neutral orientation

            // Add the pose to the path
            path.poses.push_back(pose);
            RCLCPP_INFO(this->get_logger(), "Point added to path at x: %f, y: %f", msg->point.x, msg->point.y);
        }else{
            std::cout<<"The number of points for the path is maximum. Please send the path !!!"<<std::endl;
        }

    }

    void publishPath()
    {
        if(!path.poses.empty()){
            publisher_->publish(path);
            RCLCPP_INFO(this->get_logger(), "Final path published with %zu points.", path.poses.size());
        }else{
            RCLCPP_INFO(this->get_logger(), "NO path to send.");
        }
    }
    nav_msgs::msg::Path path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
};

// kill init_pos_pub node
void kill_init_pos_pub(){

}

// kill the main process
static void* thread_handle(void *args){
    pid_t*data = (pid_t*)args;
    char str;
    sleep(1);
    std::cout<<"Press 's' or 'S' to send the path and terminate"<<std::endl;
    while (std::cin>>str)
    {
        if((str == 's') || (str == 'S')){
            break;
        }else{
            std::cout<<"WRONG CMD TO TERMINATE !!! PLEASE PRESS 'S' or 's' TO SEND THE PATH AND TERMINATE"<<std::endl;
        }
    }
    if (kill(*data, 2) == -1) {
        perror("kill");
    }
    return nullptr;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    pid_t main_pid = getpid();
    pthread_t tid;
    pthread_create(&tid,NULL, &thread_handle,&main_pid);
    pthread_detach(tid);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
