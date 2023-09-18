// C++
#include <memory>
#include <vector>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_geometry/pinhole_camera_model.h>
// ROS msgs
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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
    void publish_camera_FOV(std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_message, int id, int max_distance);
    void save_static_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg);
    void save_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg);
    void save_gt_bbox(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg);

    vector<vector<float> > ious(vector<vector<float> > &aminmaxs, vector<vector<float> > &bminmaxs);
    shape_msgs::msg::Mesh getCameraFOVMesh(const sensor_msgs::msg::CameraInfo& camera_info, double max_dist);
    visualization_msgs::msg::Marker getCameraFOVMarker(const geometry_msgs::msg::Pose& pose,
                                                                const shape_msgs::msg::Mesh& mesh, int id, std::string frame_id);
    visualization_msgs::msg::Marker getLidarRangeMarker(std::string frame_id, int id, float range);
    void publish_lidar_range(std::vector<std::string> lidars, std::vector<int64_t> lidars_max_dist);

  private:

    bool _show_range;
    int _fps;
    float _match_thresh;

    std::string _fixed_frame;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    std::vector<geometry_msgs::msg::Transform> _last_transform_camera;

    std::vector<vision_msgs::msg::BoundingBox3D> _static_objects;
    std::vector<vision_msgs::msg::BoundingBox3D> _moving_objects_bbox;
    visualization_msgs::msg::MarkerArray::SharedPtr _moving_objects;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _tracker_out_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr _static_ground_truth_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr _ground_truth_sub;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> _cameras_sub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _sensor_range_pub;
};