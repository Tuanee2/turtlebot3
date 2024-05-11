#include "../include/MyCostmapPlugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>

namespace costmap_plugin
{
    WorkAreaLayer::WorkAreaLayer() : min_x_(0), min_y_(0), max_x_(0), max_y_(0) {}

    WorkAreaLayer::~WorkAreaLayer() {}

    void WorkAreaLayer::onInitialize()
    {
        current_ = true;
        default_value_ = nav2_costmap_2d::NO_INFORMATION;
        if (node_work_area_)
        {
            polygon_subscriber_ = node_work_area_->create_subscription<geometry_msgs::msg::PolygonStamped>(
                "/work_area",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&WorkAreaLayer::polygonCallback, this, std::placeholders::_1));
        }else{
            throw std::runtime_error{"Failed to lock node"};
        }
        matchSize();
    }

    void WorkAreaLayer::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message)
    {
        transformed_polygon_.clear();
        for (const auto& p : message->polygon.points) {
            geometry_msgs::msg::Point point;
            point.x = p.x; // Chuyển đổi từ float32 của Point32 sang double của Point
            point.y = p.y;
            point.z = p.z;
            transformed_polygon_.push_back(point);
            min_x_ = std::min(min_x_,point.x);
            max_x_ = std::max(max_x_,point.x);
            min_y_ = std::min(min_y_,point.y);
            max_y_ = std::max(max_y_,point.y);
        }
    }

    void WorkAreaLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
    {
        *min_x = std::min(*min_x, this->min_x_);
        *min_y = std::min(*min_y, this->min_y_);
        *max_x = std::max(*max_x, this->max_x_);
        *max_y = std::max(*max_y, this->max_y_);
    }

    void WorkAreaLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int max_i, int min_j, int max_j)
    {
        for (int i = min_i; i < max_i; i++) {
            for (int j = min_j; j < max_j; j++) {
                double wx, wy;
                master_grid.mapToWorld(i, j, wx, wy);
                if (inside(wx, wy, transformed_polygon_)) {
                    master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
                } else {
                    master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }
    }

    bool WorkAreaLayer::inside(double x, double y,std::vector<geometry_msgs::msg::Point> polygon){
        return true;
    }

    void WorkAreaLayer::deactivate()
    {
        polygon_subscriber_ = nullptr;
    }

    void WorkAreaLayer::activate()
    {
        if (node_work_area_)
        {
            polygon_subscriber_ = node_work_area_->create_subscription<geometry_msgs::msg::PolygonStamped>(
                "/work_area",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&WorkAreaLayer::polygonCallback, this, std::placeholders::_1));
        }
    }

    void WorkAreaLayer::reset()
    {
        deactivate();
        activate();
    }

    bool WorkAreaLayer::isClearable(){
        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(costmap_plugin::WorkAreaLayer, nav2_costmap_2d::Layer)
