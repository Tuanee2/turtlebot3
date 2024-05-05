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

  void followPath()
  {
    size_t target_pose_index_ = 0;
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = current_pose.header.stamp;

    //chose the follow mode
    RCLCPP_INFO(this->get_logger(), "Press 'n' to start at the nearest pose, 'f' to start at the first pose");
    char str;
    while(std::cin>>str){
      if(str == 'n'){
        target_pose_index_ = findTheShortestWay();
        break;
      }else if(str == 'f'){
        break;
      }else{
        std::cout<<"WRONG CMD !!!"<<std::endl;
      }
    }

    while(target_pose_index_ <= plan.poses.size())
    {
      target_pose.pose = plan.poses[target_pose_index_].pose;
      
      controller(Kp_x,target_pose.pose.position.x,target_pose.pose.position.y);

      target_pose_index_++;

      if (target_pose_index_ > plan.poses.size()) 
      {
        RCLCPP_INFO(this->get_logger(), "Wait a new path!");
        break;
      }
    }
  }

  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion quat) {
        
    tf2::Quaternion q_current(quat.x,quat.y,quat.z,quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalizeAngle(double angle) {
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  void controller(double kp,double x,double y){

    while(1){
      double goal_angle = atan2(y - current_pose.pose.pose.position.y,x - current_pose.pose.pose.position.x);

      double current_angle = getYawFromQuaternion(current_pose.pose.pose.orientation);

      double angle_error = normalizeAngle(goal_angle - current_angle);

      geometry_msgs::msg::Twist cmd_vel;

      if(cond == true){
        if (angle_error >= 0.01 || angle_error <= -0.01)
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = kp*angle_error;
          publisher_->publish(cmd_vel);
        }else{
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          publisher_->publish(cmd_vel);
          cond = false;
        }
      }
      if(cond == false){
        double x_err = (x - current_pose.pose.pose.position.x);
        double y_err = (y - current_pose.pose.pose.position.y);
        pose_error = sqrt(x_err*x_err + y_err*y_err);
        if (pose_error <= 0.05 || pose_error >= -0.05)
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          publisher_->publish(cmd_vel);
          cond = true;
        }
        else
        {
          cmd_vel.linear.x = kp*pose_error;
          cmd_vel.angular.z = desire_yaw;
          publisher_->publish(cmd_vel);
        }
      }
      rclcpp::spin_some(shared_from_this());
    }
  }



  size_t findTheShortestWay(){

    double* distance = new double[plan.poses.size()-1];

    // calculate distances
    for(size_t i = 0;i < (plan.poses.size()-1);i++){
      double x_err = plan.poses[i+1].pose.position.x - plan.poses[i].pose.position.x;
      double y_err = plan.poses[i+1].pose.position.y - plan.poses[i].pose.position.y;
      double numerator = x_err*(plan.poses[i].pose.position.y - current_pose.pose.pose.position.y) - (plan.poses[i].pose.position.x -current_pose.pose.pose.position.x)*y_err;
      distance[i] = abs(numerator)/sqrt(x_err*x_err + y_err*y_err);
    }

    //find the shortest distance
    size_t shortest_index = 0;
    double shortest_distance = distance[0];
    
    for(size_t i = 1;i < (plan.poses.size()-1);i++){
      if(distance[i]<=shortest_distance){
        shortest_distance = distance[i];
        shortest_index = i;
      }
    }

    //find the nearest pose on the path
    double x1 = plan.poses[shortest_index].pose.position.x;
    double y1 = plan.poses[shortest_index].pose.position.y;
    double x2 = plan.poses[shortest_index + 1].pose.position.x;
    double y2 = plan.poses[shortest_index + 1].pose.position.y;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double t = ((current_pose.pose.pose.position.x - x1) * dx + (current_pose.pose.pose.position.y - y1) * dy) / (dx * dx + dy * dy);
    geometry_msgs::msg::PoseWithCovarianceStamped nearest_pose;
    nearest_pose.pose.pose.position.x = x1 + t * dx;
    nearest_pose.pose.pose.position.y = y1 + t * dy;

    //Go to the nearest pose
    controller(3,nearest_pose.pose.pose.position.x,nearest_pose.pose.pose.position.y);

    delete[] distance;
    return shortest_index+1;
  }


private:
// update path
  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
    // You might want to implement some logic based on the path here
    plan.poses = msg->poses;
  }
//update position
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
    node->followPath();
    rclcpp::shutdown();
  }
  return 0;
}
