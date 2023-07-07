#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
// #include "bb_interfaces/msg/detection2_d_with_features.hpp"

using namespace std::chrono_literals;

/* This Node publishes two messages with the same header simuling
 a node publishing info about bounding boxes and relative features */

class BBPublisher : public rclcpp::Node
{
  public:
    BBPublisher()
    : Node("bb_publisher"), count_(0)
    {
      det_publisher_ = this->create_publisher<vision_msgs::msg::Detection2D>("detection", 10);
      feat_publisher_ = this->create_publisher<vision_msgs::msg::Detection2D>("features", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&BBPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto detection = vision_msgs::msg::Detection2D();
      detection.bbox.center.x = 5.0;
      // auto features = std_msgs::msg::Float32MultiArray();
      auto features = vision_msgs::msg::Detection2D();
      features.bbox.center.x = -5.0;
      auto header = std_msgs::msg::Header();
      detection.header = header;
      features.header = header;
      RCLCPP_INFO(this->get_logger(), "Publishing");
      det_publisher_->publish(detection);
      feat_publisher_->publish(features);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr det_publisher_;
    rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr feat_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBPublisher>());
  rclcpp::shutdown();
  return 0;
}