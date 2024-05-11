#ifndef WORK_AREA_LAYER_HPP_
#define WORK_AREA_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include <vector>

namespace costmap_plugin
{
    class WorkAreaLayer : public nav2_costmap_2d::Layer
    {
    public:
        WorkAreaLayer();
        ~WorkAreaLayer() override;

        void onInitialize() override;
        void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
        void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int max_i, int min_j, int max_j) override;
        void deactivate() override;
        void activate() override;
        void reset() override;
        bool inside(double x, double y,std::vector<geometry_msgs::msg::Point> polygon);
        bool isClearable() override;
        //void onFootprintChanged() override;
        //void matchSize() override;
        //void onFootprintChanged() override;

    private:
        void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message);
        std::vector<geometry_msgs::msg::Point> transformed_polygon_;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_subscriber_;
        rclcpp::Node::SharedPtr node_work_area_,node_pose_;
        double min_x_, min_y_, max_x_, max_y_;
        unsigned char default_value_;
    };
}

#endif  // WORK_AREA_LAYER_HPP_
