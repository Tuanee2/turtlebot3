#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include "myCustomDisplay.hpp"
#include <vector>
#include <iostream>
#include <memory> // For smart pointers
#include <math.h>

using namespace rviz_common;

MyCustomDisplay::MyCustomDisplay() :
    color_property_(new properties::ColorProperty("Color", QColor(255, 0, 0),
                                                  "Color of the point.", this)),
    size_property_(new properties::IntProperty("Size", 10,
                                               "Size of the point.", this)),
    line_type_property_(new properties::EnumProperty("Line Type","Path","Choose between wall or path.",this)),

    workspace_status_property_(new properties::EnumProperty("Workspace status","Incomplete","Choose between Incomplete or Completed.",this)),
    
    line_typeOfShape_property_(new properties::EnumProperty("Line Type of shape", "Straight",
                                                     "Choose between a straight line or a curved line.", this)),
    line_Orientation_type_property_(new properties::EnumProperty("Orientation Type", "UpsideDown",
                                                                 "Choose between a UpsideDown type or UpRight type.",this)),
    radius_property_(new properties::FloatProperty("Radius", 1.0,
                                                   "Radius of the curve (only used if 'Curved' is selected).", this))
{
    line_type_property_->addOption("Path",0);
    line_type_property_->addOption("Wall",1);
    workspace_status_property_->addOption("Incomplete",0);
    workspace_status_property_->addOption("Completed",1);
    line_typeOfShape_property_->addOption("Straight", 0);
    line_typeOfShape_property_->addOption("Curved", 1);
    line_Orientation_type_property_->addOption("UpsideDown", 0);
    line_Orientation_type_property_->addOption("UpRight", 1);
}

MyCustomDisplay::~MyCustomDisplay() {
    for (auto& entry : points_) {
        scene_manager_->destroyManualObject(entry.second);
        scene_node_->removeAndDestroyChild(entry.first);
    }
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
        std::cerr << "Maximum number of points (100) reached." << std::endl;
        return;
    }
    
    if (!msg) {
        std::cerr << "Received invalid message." << std::endl;
        return;
    }

    
    std::string type;
    if(line_type_property_->getOptionInt() == 0){
        type = "Path";
        point_positions_path.push_back(Ogre::Vector3(msg->point.x, msg->point.y, 0));
        if(workspace_status_property_->getOptionInt() == 0){
            createPoint(msg->point,type);
            connectPoints(type);
        }else{
            if(checkPoint(msg->point) == false){
                std::cout<<"Wrong pose, please clicked inside the Work Area."<<std::endl;
            }else{
                createPoint(msg->point,type);
                connectPoints(type);
            }
        }
    }else{
        type = "Wall";

        if(workspace_status_property_->getOptionInt() == 0){
            point_positions_wall.push_back(Ogre::Vector3(msg->point.x, msg->point.y, 0));
            createPoint(msg->point,type);
        }else{
            if (!point_positions_wall.empty()) {
                point_positions_wall.push_back(Ogre::Vector3(point_positions_wall.front().x, point_positions_wall.front().y, 0));
            }
        }
        connectPoints(type);
    }
}

void MyCustomDisplay::createPoint(const geometry_msgs::msg::Point& point, const std::string& type) {
    Ogre::SceneNode* node = scene_node_->createChildSceneNode();
    Ogre::ManualObject* pointObj = scene_manager_->createManualObject();
    
    pointObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
    pointObj->position(point.x, point.y, 0);
    pointObj->colour(color_property_->getOgreColor());
    pointObj->end();
    
    pointObj->setMaterial(0, createOrGetPointMaterial(type));
    node->attachObject(pointObj);
    points_[node] = pointObj;
}

Ogre::MaterialPtr MyCustomDisplay::createOrGetPointMaterial(const std::string& type) {
    std::string materialName = "PointMaterial_" + type;
    if (Ogre::MaterialManager::getSingleton().resourceExists(materialName)) {
        return Ogre::MaterialManager::getSingleton().getByName(materialName);
    } else {
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
            materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->setReceiveShadows(false);
        material->getTechnique(0)->getPass(0)->setDiffuse(color_property_->getOgreColor());
        material->getTechnique(0)->getPass(0)->setAmbient(color_property_->getOgreColor());
        material->getTechnique(0)->getPass(0)->setPointSize(size_property_->getInt());
        return material;
    }
}

void MyCustomDisplay::connectPoints(const std::string& type) {

    std::vector<Ogre::Vector3>& points = (type == "Path") ? point_positions_path : point_positions_wall;
    std::vector<Ogre::ManualObject*>& lines = (type == "Path") ? lines_path : lines_wall;

    if (points.size() < 2) {
        return;
    }

    for (const auto& line : lines) {
        scene_manager_->destroyManualObject(line);
    }
    lines.clear();

    for (size_t i = 0; i < points.size() - 1; ++i) {
        const Ogre::Vector3& p1 = points[i];
        const Ogre::Vector3& p2 = points[i + 1];
        
        Ogre::ManualObject* line = scene_manager_->createManualObject();
        line->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

        if (line_typeOfShape_property_->getOptionInt() == 0) { // Straight line
            line->position(p1);
            line->position(p2);
            
        } else { // Curved line
            drawCurvedLine(line, p1, p2);
        }

        line->colour(color_property_->getOgreColor());
        line->end();
        
        line->setMaterial(0, createOrGetLineMaterial(type));
        scene_node_->attachObject(line);
        lines.push_back(line);
    }
}

void MyCustomDisplay::drawCurvedLine(Ogre::ManualObject* line, const Ogre::Vector3& start, const Ogre::Vector3& end) {
    Ogre::Real radius = radius_property_->getFloat();
    Ogre::Real distance = start.distance(end);
    Ogre::Vector3 mid_point = (start + end) / 2.0f;
    Ogre::Real real_radius = Ogre::Math::Sqrt(radius * radius + distance * distance/4);
    Ogre::Radian angle_radian = Ogre::Math::ACos(radius/real_radius);
    Ogre::Real angle_inc = angle_radian.valueRadians() / 20.0f;
    // Tính toán tâm của cung tròn
    Ogre::Vector3 direction = end - start;
    direction.normalise();
    Ogre::Vector3 center;
    if(line_Orientation_type_property_->getOptionInt() == 0 ){
      center = mid_point + radius * direction.crossProduct(Ogre::Vector3::UNIT_Z);
    }else{
      center = mid_point - radius * direction.crossProduct(Ogre::Vector3::UNIT_Z);
    }

    Ogre::Vector3 vec_from_center_to_start = start - center;
    Ogre::Vector3 vec_from_center_to_end = end - center;
    std::cout<<std::endl;
    std::cout<<"vec_start : "<<vec_from_center_to_start.x <<" : "<<vec_from_center_to_start.y<<std::endl;
    std::cout<<"vec_end : "<<vec_from_center_to_end.x <<" : "<<vec_from_center_to_end.y<<std::endl;
    
    vec_from_center_to_start.normalise();
    vec_from_center_to_end.normalise();

    // Tính góc giữa hai vectơ và trục x bằng hàm ACos
    Ogre::Radian angle_start_to_center_x = Ogre::Math::ATan(vec_from_center_to_start.y/vec_from_center_to_start.x);
    Ogre::Radian angle_end_to_center_x = Ogre::Math::ATan(vec_from_center_to_end.y/vec_from_center_to_end.x);
    std::cout<<"test"<<std::endl;
    Ogre::Real angle_start = angle_start_to_center_x.valueRadians();
    Ogre::Real angle_end = angle_end_to_center_x.valueRadians();
    std::cout<<"test01"<<std::endl;
    //**********debug*********
    std::cout<<start.x <<" : "<<end.x<<std::endl;
    std::cout<<"Center : "<<center.x <<" : "<<center.y<<std::endl;
    std::cout<<"vec_start : "<<vec_from_center_to_start.x <<" : "<<vec_from_center_to_start.y<<std::endl;
    std::cout<<"vec_end : "<<vec_from_center_to_end.x <<" : "<<vec_from_center_to_end.y<<std::endl;
    std::cout<<angle_start*180/Ogre::Math::PI<<" : "<<angle_end*180/Ogre::Math::PI<<std::endl;
    //*************************************
    Ogre::Real angle_swip = 0.0f;
    if(((angle_end - angle_start) <= Ogre::Math::PI) || ((angle_end - angle_start) >= -Ogre::Math::PI)){
      if(angle_end - angle_start > 0){

      }else{
        angle_swip = angle_end;
        angle_end = angle_start;
        angle_start = angle_swip;
      }
    }else{
      if(angle_end - angle_start > 0){
        angle_swip = angle_end;
        angle_end = angle_start;
        angle_start = angle_swip;
      }else{

      }
    }
  
    std::cout<<angle_start*180/Ogre::Math::PI<<" : "<<angle_end*180/Ogre::Math::PI<<std::endl;
        
    for (Ogre::Real angle = angle_start; angle <= angle_end ; angle += angle_inc) {
        Ogre::Real x = center.x + real_radius * Ogre::Math::Cos(angle);
        Ogre::Real y = center.y + real_radius * Ogre::Math::Sin(angle);
        Ogre::Real z = center.z;
        Ogre::Vector3 point(x, y, z);
          // Thêm điểm vào đường nối
        line->position(point);
    }
    

}

Ogre::MaterialPtr MyCustomDisplay::createOrGetLineMaterial(const std::string& type) {
    std::string materialName = "LineMaterial_" + type;

    if (Ogre::MaterialManager::getSingleton().resourceExists(materialName)) {
        return Ogre::MaterialManager::getSingleton().getByName(materialName);
    }

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
        materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setReceiveShadows(false);
    if(type == "Wall"){
        material->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    }else{
        material->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 0.0f));
    }
    material->getTechnique(0)->getPass(0)->setLineWidth(20.0f);
    return material;
}

bool MyCustomDisplay::checkPoint(const geometry_msgs::msg::Point& point){

    int i, j, nvert = point_positions_wall.size();
        bool inside = false;

        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            if (((point_positions_wall[i].y >= point.y) != (point_positions_wall[j].y >= point.y)) &&
                (point.x <= (point_positions_wall[j].x - point_positions_wall[i].x) * (point.y - point_positions_wall[i].y) / (point_positions_wall[j].y - point_positions_wall[i].y) + point_positions_wall[i].x))
                inside = !inside;
        }

        return inside;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MyCustomDisplay, rviz_common::Display)
