#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <pthread.h>
#include <signal.h>
#include <chrono>
#include <functional>
#include <memory>
#include <math.h>

bool cond = true;


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

  void controller()
  {
    size_t target_pose_index_ = 0;
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = current_pose.header.stamp;

    while(target_pose_index_ <= plan.poses.size())
    {
      target_pose.pose = plan.poses[target_pose_index_].pose;
      //error_yaw
      double roll, pitch;
      // get current_yaw
      tf2::Quaternion q_current(
        current_pose.pose.pose.orientation.x,
        current_pose.pose.pose.orientation.y,
        current_pose.pose.pose.orientation.z,
        current_pose.pose.pose.orientation.w
      );
      tf2::Matrix3x3 n(q_current);
      n.getRPY(roll, pitch, current_yaw);
      // calculate target_yaw
      target_yaw = atan2(target_pose.pose.position.y - current_pose.pose.pose.position.y,target_pose.pose.position.x - current_pose.pose.pose.position.x);
      desire_yaw = target_yaw - current_yaw;
      // limit desire_yaw
      if (desire_yaw  > M_PI) {
        desire_yaw  -= 2 * M_PI;
      }else if (desire_yaw  < -M_PI) {
        desire_yaw  += 2 * M_PI;
      }
      // P controller
      if(cond == true){
        if (desire_yaw >= 0.01 || desire_yaw <= -0.01)
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = desire_yaw;
          publisher_->publish(cmd_vel);
        }else{
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          publisher_->publish(cmd_vel);
          cond = false;
        }
      }
      if(cond == false){
        if (pose_error <= 0.05 || pose_error >= -0.05)
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          publisher_->publish(cmd_vel);
          target_pose_index_++;
          cond = true;
        }
        else
        {
          double x_err = (target_pose.pose.position.x - current_pose.pose.pose.position.x);
          double y_err = (target_pose.pose.position.y - current_pose.pose.pose.position.y);
          pose_error = sqrt(x_err*x_err + y_err*y_err);
          cmd_vel.linear.x = Kp_x*pose_error;
          cmd_vel.angular.z = desire_yaw;
          publisher_->publish(cmd_vel);
        }
      }
    
      if (target_pose_index_ > plan.poses.size()) 
      {
        RCLCPP_INFO(this->get_logger(), "Wait a new path!");
        break;
      }
      rclcpp::spin_some(shared_from_this());
    }
  }


private:
  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
    // You might want to implement some logic based on the path here
    plan.poses = msg->poses;
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose: [%f, %f, %f]",
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    // Process the pose information and potentially publish to /cmd_vel
    current_pose.pose = msg->pose;
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist cmd_vel;
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose;
  nav_msgs::msg::Path plan;
  rclcpp::TimerBase::SharedPtr timer_;
  double current_yaw , target_yaw, desire_yaw, pose_error;
  float Kp_x = 3;
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
    auto node = std::make_shared<PlanPoseToCmdVelNode>();
    node->controller();
    rclcpp::shutdown();
  }
  return 0;
}
