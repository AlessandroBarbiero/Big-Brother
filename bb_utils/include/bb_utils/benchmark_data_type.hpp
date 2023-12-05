#pragma once

#include <cstddef>
#include <vector>
#include <unordered_map>

#include <vision_msgs/msg/bounding_box3_d.hpp>

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

  typedef struct obj{
    int id;
    vision_msgs::msg::BoundingBox3D bbox;
    ClassLabel label;
  } Object3D;

}