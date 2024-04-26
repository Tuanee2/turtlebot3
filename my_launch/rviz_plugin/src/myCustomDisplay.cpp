#include "myCustomDisplay.hpp"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <vector>
#include <iostream>
#include <math.h>

using namespace rviz_common;

MyCustomDisplay::MyCustomDisplay() {
  color_property_ = new properties::ColorProperty("Color", QColor(255, 0, 0),
                                                  "Color of the point.", this);
  size_property_ = new properties::IntProperty("Size", 10,
                                               "Size of the point.", this);
  line_type_property_ = new properties::EnumProperty("Line Type", "Straight",
                                                     "Choose between a straight line or a curved line.", this);
  line_type_property_->addOption("Straight", 0);
  line_type_property_->addOption("Curved", 1);
  radius_property_ = new properties::FloatProperty("Radius", 1.0,
                                                   "Radius of the curve (only used if 'Curved' is selected).", this);

}

MyCustomDisplay::~MyCustomDisplay() {
  for (auto& entry : points_) {
    scene_manager_->destroyManualObject(entry.second);
    scene_node_->removeAndDestroyChild(entry.first);
  }
  // Delete the properties if they were created with new
  delete color_property_;
  delete size_property_;
  delete line_type_property_;
  delete radius_property_;
}

void MyCustomDisplay::onInitialize() {
  RosTopicDisplay::onInitialize();
}

void MyCustomDisplay::onEnable() {
  subscribe();
}

void MyCustomDisplay::onDisable() {
  unsubscribe();
}

void MyCustomDisplay::subscribe() {
  RosTopicDisplay::subscribe();
}

void MyCustomDisplay::unsubscribe() {
  RosTopicDisplay::unsubscribe();
}

// main process
void MyCustomDisplay::processMessage(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  if (points_.size() >= 100) {
    std::cout << "Đã đạt tối đa số điểm có thể nhấn là 100." << std::endl;
    return; // Ngừng xử lý và không thêm điểm mới
  }
  // add a new point position to "point_position_" Vector
  point_positions_.push_back(Ogre::Vector3(msg->point.x, msg->point.y, 0));
  Ogre::SceneNode* node = scene_node_->createChildSceneNode();
  Ogre::ManualObject* point = scene_manager_->createManualObject();
  Ogre::MaterialPtr material;
  if (Ogre::MaterialManager::getSingleton().resourceExists("PointMaterial")) {
      material = Ogre::MaterialManager::getSingleton().getByName("PointMaterial");
  } else {
      material = Ogre::MaterialManager::getSingleton().create(
          "PointMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setReceiveShadows(false);
      material->getTechnique(0)->getPass(0)->setDiffuse(color_property_->getOgreColor());
      material->getTechnique(0)->getPass(0)->setAmbient(color_property_->getOgreColor());
      material->getTechnique(0)->getPass(0)->setPointSize(size_property_->getInt());

      line_material_ = Ogre::MaterialManager::getSingleton().create(
          "LineMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      line_material_->setReceiveShadows(false);
      // Bạn có thể tùy chỉnh các thuộc tính hiển thị của đường tại đây (ví dụ như màu sắc, độ dày)
      line_material_->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 0.0f)); 
      line_material_->getTechnique(0)->getPass(0)->setLineWidth(5.0f); // Độ dày đường là 2
  }
  point->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
  point->position(msg->point.x, msg->point.y, 0);
  point->colour(color_property_->getOgreColor());
  point->end();
  point->setMaterial(0, material);
  node->attachObject(point);
  MyCustomDisplay::connectPoints();
}

void MyCustomDisplay::connectPoints() {
  // Method to connect points based on the selected line type and configuration
  // Clear existing lines (if any)
  for (const auto& line : lines_) {
    scene_manager_->destroyManualObject(line);
  }
  lines_.clear();

  // Check if there are enough points (at least 2) to draw a line
  if (point_positions_.size() < 2) {
    return;
  }

  // Iterate through consecutive pairs of points
  for (size_t i = 0; i < point_positions_.size() - 1; ++i) {
    const Ogre::Vector3& p1 = point_positions_[i];
    const Ogre::Vector3& p2 = point_positions_[i + 1];

    Ogre::ManualObject* line = scene_manager_->createManualObject();
    line->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

    if (line_type_property_->getOptionInt() == 0) { // Straight line
      line->position(p1);
      line->position(p2);
    } else { // Curved line (assuming a simple arc)
      Ogre::Real radius = radius_property_->getFloat();
      Ogre::Real radius1 =  sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y)) + radius;
      Ogre::Vector3 center = (p1 + p2) / 2.0f;
      Ogre::Vector3 direction = p2 - p1;
      direction.normalise();

      // Calculate start and end points for the arc based on direction
      Ogre::Vector3 start_point = center + direction.perpendicular() * radius1;
      Ogre::Vector3 end_point = center - direction.perpendicular() * radius1;

      // Draw an arc with appropriate number of segments
      const int segments = 20; // Adjust as needed for smoothness
      for (int j = 0; j <= segments; ++j) {
        Ogre::Real angle = j * M_PI / segments;
        Ogre::Vector3 point = center + radius * direction.perpendicular() * cos(angle) + radius * direction.crossProduct(direction.perpendicular()) * sin(angle);
        line->position(point);
      }
    }

    line->colour(color_property_->getOgreColor());
    line->end();

    // Set material and attach to scene node
    line->setMaterial(0, line_material_); // Assuming a shared material for lines
    scene_node_->attachObject(line);
    lines_.push_back(line);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MyCustomDisplay, rviz_common::Display)

