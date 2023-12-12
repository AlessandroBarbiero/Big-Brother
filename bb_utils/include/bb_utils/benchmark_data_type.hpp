#pragma once

#include <cstddef>
#include <vector>
#include <unordered_map>
#include <sstream>

#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace bb_bench {
  enum class ClassLabel{
      Unknown = 0,
      Pedestrian, 
      Bicycle,
      Motorcycle, 
      Car
      };

  static const std::string classLabelString[] = {"Unknown", "Person", "Cyclist", "Motorcycle", "Car"};

  static const std::unordered_map<std::string, ClassLabel> class_to_label = {
    {"", ClassLabel::Unknown},
    {"unknown", ClassLabel::Unknown},
    {"other", ClassLabel::Unknown},
    {"person", ClassLabel::Pedestrian},
    {"pedestrian", ClassLabel::Pedestrian},
    {"bicycle", ClassLabel::Bicycle},
    {"cyclist", ClassLabel::Bicycle},
    {"motorcycle", ClassLabel::Motorcycle},
    {"vehicle", ClassLabel::Car},
    {"car", ClassLabel::Car},
    {"truck", ClassLabel::Car}
  };

  // Keep angle within [-PI , PI]
  inline void normalizeAngle(float& angle){
    angle = fmod(angle + M_PI, 2*M_PI) - M_PI;
  }

  float getYawFromQuat(geometry_msgs::msg::Quaternion& quat){
    tf2::Quaternion q(
          quat.x,
          quat.y,
          quat.z,
          quat.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw_d;
      m.getRPY(roll, pitch, yaw_d, 2);
    float yaw = yaw_d;
    // Keep yaw within [-PI , PI]
    normalizeAngle(yaw);
    return yaw * 180.0 / M_PI;
  }

  typedef struct obj{
    int id;
    ClassLabel label;
    vision_msgs::msg::BoundingBox3D bbox;

    std::string toCSV(){
      std::ostringstream ss;
      ss << id << "," << classLabelString[static_cast<int>(label)] << "," << 
        bbox.center.position.x << "," << bbox.center.position.y << "," << bbox.center.position.z << "," <<
        getYawFromQuat(bbox.center.orientation) << "," << 
        bbox.size.x << "," << bbox.size.y << "," << bbox.size.z;
      return ss.str();
    }
  } Object3D;

}