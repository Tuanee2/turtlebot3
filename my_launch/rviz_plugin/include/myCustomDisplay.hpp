#ifndef MY_CUSTOM_DISPLAY_HPP_
#define MY_CUSTOM_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
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
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::IntProperty* size_property_;
  std::vector<std::pair<Ogre::SceneNode*, Ogre::ManualObject*>> points_;
};

#endif // MY_CUSTOM_DISPLAY_HPP_
