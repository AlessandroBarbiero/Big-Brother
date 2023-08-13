// C++
#include <memory>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ROS msgs
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// extra
#include <opencv2/opencv.hpp>
#include <bb_tracker/BYTETracker.h>

using std::placeholders::_1;

// Node that tracks the bounding boxes 3D published in a topic using Byte track

class BBTracker : public rclcpp::Node
{
  public:
    BBTracker();
    /**
    * Transforms the detections in the input old_message from the old_frame to the new_frame.
    * The function calculates the transformation and applies it to the bounding box positions, orientations, and dimensions.
    * If any error occurs while obtaining the transform, a message is printed, and the function returns.
    * This operation is done in place, meaning the original old_message->detections are modified with the transformed values.
    * @param old_message The shared pointer to the input vision_msgs::msg::Detection3DArray containing the detections to transform.
    * @param new_frame The std::string& representing the new_frame to which the detections are transformed.
    */
    void change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, std::string& new_frame);

  private:
    void decode_detections(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message, vector<Object>& objects);
    void add_detection(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message);
    void periodic_update();
    void publish_stracks(vector<STrack*>& output_stracks);
    visualization_msgs::msg::Marker createPathMarker(STrack* track, std_msgs::msg::Header& header, geometry_msgs::msg::Point& last_point, visualization_msgs::msg::Marker& text);

  private:

    std::string _fixed_frame;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _det_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _det_poses_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _text_publisher;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _detection;


    BYTETracker _tracker;
    vector<Object> _objects_buffer;
    // Total number of detections received
    int _num_detections;
    // Total number of BYTETracker updates
    int _num_updates;
    // Microseconds passed computing the algorithm
    long unsigned int _total_ms;
};