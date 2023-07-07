#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

// using std::placeholders::_1;
// using std::placeholders::_2;


// Node that subscribe to two synchronized topics and print when received a message on both

class BBSubscriber : public rclcpp::Node
{
  public:
    BBSubscriber()
    : Node("bb_subscriber")
    {
      
      detection_sub.subscribe(this, "detection");
      features_sub.subscribe(this, "features");

      sync_ = std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2D, vision_msgs::msg::Detection2D>>(
      detection_sub, features_sub, 10);

      // Register the callback function
      sync_->registerCallback(std::bind(&BBSubscriber::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

      // subscription_ = this->create_subscription<std_msgs::msg::String>(
      // "topic", 10, std::bind(&BBSubscriber::topic_callback, this, _1));
    }

  private:
    void sync_callback(const vision_msgs::msg::Detection2D::ConstSharedPtr& detection, const vision_msgs::msg::Detection2D::ConstSharedPtr& features) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", detection->bbox);
    }
    message_filters::Subscriber<vision_msgs::msg::Detection2D> detection_sub;
    message_filters::Subscriber<vision_msgs::msg::Detection2D> features_sub;
    std::shared_ptr<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2D, vision_msgs::msg::Detection2D>> sync_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBSubscriber>());
  rclcpp::shutdown();
  return 0;
}