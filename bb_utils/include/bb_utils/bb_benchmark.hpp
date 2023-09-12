// C++
#include <memory>
#include <vector>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ROS msgs
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;
using namespace std;

// Node that compute statistics for tracking algorithm considering the ground truth

class BBBenchmark : public rclcpp::Node
{
  public:
    BBBenchmark();

    void change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, std::string& new_frame);


  private:
    void tracker_out(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message);
    vector<vector<float> > ious(vector<vector<float> > &aminmaxs, vector<vector<float> > &bminmaxs);

    //   void camera_callback(const vision_msgs::msg::Detection2D::ConstSharedPtr& detection, const vision_msgs::msg::Detection2D::ConstSharedPtr& features) const
    // {
    //   shape_msgs::Mesh mesh = getCameraFOVMesh(*camera_info_, calibration_display_->fov_marker_size_property_->getFloat());
    //       visual_tools_->setBaseFrame(to_frame.toStdString());
    //       visual_tools_->setAlpha(calibration_display_->fov_marker_alpha_property_->getFloat());
    //       visual_tools_->publishMesh(fov_pose_, mesh, rvt::YELLOW, 1.0, "fov", 1);
    // }

  private:
    bool _show_range;
    int _fps;
    float _match_thresh;
    std::string _fixed_frame;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _tracker_out;
};