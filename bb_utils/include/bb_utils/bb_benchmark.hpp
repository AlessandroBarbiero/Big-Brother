// C++
#include <memory>
#include <vector>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <bb_interfaces/msg/stats.hpp>

#include <bb_utils/benchmark_data_type.hpp>

#define NANOSEC_IN_SEC 1000000000

using std::placeholders::_1;
using namespace std;

// Node that compute statistics for tracking algorithm considering the ground truth

class BBBenchmark : public rclcpp::Node
{
  public:
    BBBenchmark();
    ~BBBenchmark();

    void change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, std::string& new_frame);
    void change_frame(std::shared_ptr<geometry_msgs::msg::PointStamped> old_message, std::string& new_frame);

  private:
    void compute_stats(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message);
    void append_csv(std::shared_ptr<vision_msgs::msg::Detection3DArray> tracked_objects, std::vector<bb_bench::Object3D>& objects_on_sight);
    void camera_info_callback(std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_message, int id, int max_distance);
    void save_static_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg);
    void save_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg);
    void save_gt_bbox(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg);
    void point_clicked(std::shared_ptr<geometry_msgs::msg::PointStamped> cp_message);
    void publish_selected_obj(std::vector<vision_msgs::msg::Detection3D>& tracked_obj);

    void show_objects(vector<bb_bench::Object3D> objects, std::string ns);

    vector<bb_bench::Object3D> filter_camera(vector<bb_bench::Object3D>& objects);
    vector<bb_bench::Object3D> filter_lidar(vector<bb_bench::Object3D>& objects);

    vector<vector<float> > ious(vector<vector<float> > &aminmaxs, vector<vector<float> > &bminmaxs);
    void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
	                                      vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
    double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
	                            bool extend_cost, float cost_limit = LONG_MAX, bool return_cost = true);
    vector<vector<float> > iou_distance(vector<bb_bench::Object3D> &a_bboxs, vector<vision_msgs::msg::Detection3D> &b_bboxs);

    shape_msgs::msg::Mesh getCameraFOVMesh(const sensor_msgs::msg::CameraInfo& camera_info, double max_dist);
    visualization_msgs::msg::Marker getCameraFOVMarker(const geometry_msgs::msg::Pose& pose,
                                                                const shape_msgs::msg::Mesh& mesh, int id, std::string frame_id);
    visualization_msgs::msg::Marker getLidarRangeMarker(std::string frame_id, int id, float range);
    void publish_lidar_range(std::vector<std::string> lidars, std::vector<int64_t> lidars_max_dist);
    void init_lidar_tf();
    
    vector<bb_bench::Object3D> obj_union(vector<bb_bench::Object3D>& list_a, vector<bb_bench::Object3D>& list_b);

  private:

    bool _show_range;
    int _fps;
    float _match_thresh;
    float _alpha_range;

    int _tot_false_positive = 0, _tot_true_positive = 0, _tot_missed = 0, _tot_objects_to_detect = 0;
    int _tot_ass_mismatch = 0;
    float _tot_iou_detections = 0, _tot_iou_dist_detections = 0;

    // %%%%%% CSV-FILE
    std::string _gt_file_name = "data/gt.csv";
    std::string _track_file_name = "data/track.csv";
    std::ofstream _gt_file;
    std::ofstream _track_file; 

    // %%%%%% ASSOCIATION
    std::map<int,std::string>_gt_index_to_track_id;
    int _selected_gt_id;

    // %%%%%% REGARDING TF
    std::string _fixed_frame;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    std::vector<geometry_msgs::msg::Transform> _last_transform_camera; //Used to check if it is necessary to update the saved values
    std::vector<geometry_msgs::msg::Transform> _last_transform_lidar;
    std::vector<tf2::Transform> _tf2_transform_camera; //The saved values used to compute the transformation
    std::vector<tf2::Transform> _tf2_transform_lidar;
    bool _lidar_ready;

    // %%%%%%% SENSORS DATA
    std::vector<std::string> _cameras;
    std::vector<int64_t> _cameras_max_dist;
    std::vector<image_geometry::PinholeCameraModel> _camera_models;
    std::vector<std::string> _lidars;
    std::vector<int64_t> _lidars_max_dist;

    // %%%%%%% GROUND TRUTH
    std::vector<bb_bench::Object3D> _static_objects;
    std::vector<bb_bench::Object3D> _moving_objects_bbox;
    std::vector<bb_bench::Object3D> _all_objects;
    visualization_msgs::msg::MarkerArray::SharedPtr _moving_objects;

    // %%%%% SUBSCRIBERS
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _tracker_out_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr _static_ground_truth_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr _ground_truth_sub;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> _cameras_sub;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _clicked_point_sub;

    // %%%%% PUBLISHERS
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _sensor_range_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _debug_pub;
    rclcpp::Publisher<bb_interfaces::msg::Stats>::SharedPtr _stats_pub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _selected_gt_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _selected_track_pub;
};