#ifndef MY_CUSTOM_DISPLAY_HPP_
#define MY_CUSTOM_DISPLAY_HPP_

#include <memory>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class MyCustomDisplay : public rviz_common::RosTopicDisplay<geometry_msgs::msg::PointStamped>
{
Q_OBJECT
public:
  MyCustomDisplay();
  virtual ~MyCustomDisplay() override;

protected:
  virtual void onInitialize() override;
  virtual void onEnable() override;
  virtual void onDisable() override;
  virtual void subscribe() override;
  virtual void unsubscribe() override;
  virtual void processMessage(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) override;

private:
  void createPoint(const geometry_msgs::msg::Point& point,const std::string& type);
  void connectPoints(const std::string& type);
  void drawCurvedLine(Ogre::ManualObject* line, const Ogre::Vector3& start, const Ogre::Vector3& end);
  bool checkPoint(const geometry_msgs::msg::Point& point);
  Ogre::MaterialPtr createOrGetPointMaterial(const std::string& type);
  Ogre::MaterialPtr createOrGetLineMaterial(const std::string& type);
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
  std::unique_ptr<rviz_common::properties::IntProperty> size_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> line_type_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> workspace_status_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> line_typeOfShape_property_;
  std::unique_ptr<rviz_common::properties::EnumProperty> line_Orientation_type_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> radius_property_;
  Ogre::MaterialPtr line_material_;
  std::vector<Ogre::Vector3> point_positions_path;
  std::vector<Ogre::Vector3> point_positions_wall;
  std::map<Ogre::SceneNode*, Ogre::ManualObject*> points_;
  std::vector<Ogre::ManualObject*> lines_path;
  std::vector<Ogre::ManualObject*> lines_wall;
};

#endif // MY_CUSTOM_DISPLAY_HPP_
