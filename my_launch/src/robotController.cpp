#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <pthread.h>
#include <signal.h>

class PlanPoseToCmdVelNode : public rclcpp::Node
{
public:
  PlanPoseToCmdVelNode() : Node("controller_node")
  {
    // Subscriber for /plan
    subscription_plan_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&PlanPoseToCmdVelNode::plan_callback, this, std::placeholders::_1));

    // Subscriber for /amcl_pose
    subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&PlanPoseToCmdVelNode::pose_callback, this, std::placeholders::_1));

    // Publisher for /cmd_vel
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Star follow the path.");
  }

private:
  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
    // You might want to implement some logic based on the path here
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose: [%f, %f, %f]",
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    // Process the pose information and potentially publish to /cmd_vel
    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear.x = 0.5;  // Example: move forward with a predefined speed
    cmd_vel.angular.z = 0.0; // Example: no rotation

    publisher_->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "Published cmd_vel to move forward");
  }

  void controller(){

  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  pid_t child_pid = fork();
  if(child_pid == 0){
    sleep(1);
    std::cout<<"PRESS 'p' to PAUSE, 'c' to contunue, 's' to TERMINATE."<<std::endl;
    char str;
    while(std::cin>>str){
      if((str == 'p') || (str == 'P')){
        std::cout<<"PAUSE THE PROCESS"<<std::endl;
        if (kill(getppid(), SIGSTOP) == -1) {
          perror("pause");
        }
      }else if((str == 'c') || (str == 'C')){
        std::cout<<"CONTINUE FOLOWING THE PATH. "<<std::endl;
        if(kill(getppid(),SIGCONT) == -1) {
          perror("continue");
        }
      }else if((str == 's') || (str == 'S')){
        std::cout<<"STOP FOLLOW THE PATH !!!"<<std::endl;
        if (kill(getppid(), SIGINT) == -1) {
          perror("kill");
        }
        break;
      }else{
        std::cout<<"WRONG CMD !!!"<<std::endl;
      }
    }
  }else{
    rclcpp::spin(std::make_shared<PlanPoseToCmdVelNode>());
    rclcpp::shutdown();
  }
  return 0;
}
