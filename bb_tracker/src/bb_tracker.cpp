#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <opencv2/opencv.hpp>
#include <bb_tracker/BYTETracker.h>

using std::placeholders::_1;

// Node that tracks the bounding boxes 3D published in a topic using Byte track

class BBTracker : public rclcpp::Node
{
  public:
    BBTracker()
    : Node("bb_tracker")
    {
        _detection = this->create_subscription<vision_msgs::msg::Detection3DArray>(
                "detection_3d", 10, std::bind(&BBTracker::add_detection, this, _1));
    
        RCLCPP_INFO(this->get_logger(), "Ready to track");
    }

  private:
    void add_detection(std::shared_ptr<vision_msgs::msg::Detection3DArray> detection) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detection->header.frame_id.c_str());
    }
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _detection;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBTracker>());
  rclcpp::shutdown();
  return 0;
}