#include "myCustomDisplay.hpp"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <vector>
#include <iostream>

using namespace rviz_common;

MyCustomDisplay::MyCustomDisplay() {
  color_property_ = new properties::ColorProperty("Color", QColor(255, 0, 0),
                                                  "Color of the point.", this);
  size_property_ = new properties::IntProperty("Size", 10,
                                               "Size of the point.", this);
}

MyCustomDisplay::~MyCustomDisplay() {
  for (auto& entry : points_) {
    scene_manager_->destroyManualObject(entry.second);
    scene_node_->removeAndDestroyChild(entry.first);
  }
  // Delete the properties if they were created with new
  delete color_property_;
  delete size_property_;
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

void MyCustomDisplay::processMessage(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  if (points_.size() >= 100) {
    std::cout << "Đã đạt tối đa số điểm có thể nhấn là 100." << std::endl;
    return; // Ngừng xử lý và không thêm điểm mới
  }
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
  }
  point->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
  point->position(msg->point.x, msg->point.y, msg->point.z);
  point->colour(color_property_->getOgreColor());
  point->end();
  point->setMaterial(0, material);
  node->attachObject(point);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MyCustomDisplay, rviz_common::Display)

