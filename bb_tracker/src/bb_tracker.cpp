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
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
// extra
#include <opencv2/opencv.hpp>
#include <bb_tracker/BYTETracker.h>

using std::placeholders::_1;

// #define DEBUG

// Node that tracks the bounding boxes 3D published in a topic using Byte track

class BBTracker : public rclcpp::Node
{
  public:
    BBTracker()
    : Node("bb_tracker"), _tf_buffer(this->get_clock()), _tf_listener(_tf_buffer)
    {
        this->declare_parameter("fps", 30);
        this->declare_parameter("fixed_frame", "sensors_home");

        _fixed_frame = get_parameter("fixed_frame").as_string();
        int fps = get_parameter("fps").as_int();

        _detection = this->create_subscription<vision_msgs::msg::Detection3DArray>(
                "detection_3d", 10, std::bind(&BBTracker::add_detection, this, _1));

        _det_publisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("bytetrack/detections", 10);

        _tracker = BYTETracker(fps, 30);
        _num_detections = 0;
        _total_ms = 0;
    
        RCLCPP_INFO(this->get_logger(), "Ready to track");
    }

    /**
    * Transforms the detections in the input old_message from the old_frame to the new_frame.
    * The function calculates the transformation and applies it to the bounding box positions, orientations, and dimensions.
    * If any error occurs while obtaining the transform, a message is printed, and the function returns.
    * This operation is done in place, meaning the original old_message->detections are modified with the transformed values.
    * @param old_message The shared pointer to the input vision_msgs::msg::Detection3DArray containing the detections to transform.
    * @param new_frame The std::string& representing the new_frame to which the detections are transformed.
    */
    void change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, std::string& new_frame){
      std::string old_frame = old_message->header.frame_id;
      geometry_msgs::msg::TransformStamped tf_result;
      try {
        tf_result = _tf_buffer.lookupTransform(new_frame, old_frame, rclcpp::Time(0));
      } catch (tf2::TransformException& ex) {
        std::cout << "No transform exists for the given tfs" << std::endl;
        return;
      }
      tf2::Quaternion q(
        tf_result.transform.rotation.x,
        tf_result.transform.rotation.y,
        tf_result.transform.rotation.z,
        tf_result.transform.rotation.w
      );
      tf2::Vector3 p(
        tf_result.transform.translation.x,
        tf_result.transform.translation.y,
        tf_result.transform.translation.z
      );
      tf2::Transform transform(q, p);

      // for(auto& detect : old_message->detections){
      //   auto bbox = detect.bbox;
      //   std::cout << "bbox BEFORE transform values: " 
      //               << " x=" << bbox.center.position.x
      //               << " y=" << bbox.center.position.y
      //               << " z=" << bbox.center.position.z
      //               << " w=" << bbox.size.x
      //               << " d=" << bbox.size.y
      //               << " h=" << bbox.size.z
      //               << std::endl;
      // }

      for(auto& detect : old_message->detections){

        tf2::Vector3 v(detect.bbox.center.position.x, detect.bbox.center.position.y, detect.bbox.center.position.z);
        v = transform * v;
        detect.bbox.center.position.x = v.x();
        detect.bbox.center.position.y = v.y();
        detect.bbox.center.position.z = v.z();

        tf2::Quaternion quat(detect.bbox.center.orientation.x, detect.bbox.center.orientation.y, detect.bbox.center.orientation.z, detect.bbox.center.orientation.w);
        quat = transform * quat;
        detect.bbox.center.orientation.x = quat.x();
        detect.bbox.center.orientation.y = quat.y();
        detect.bbox.center.orientation.z = quat.z();
        detect.bbox.center.orientation.w = quat.w();

        tf2::Vector3 s(detect.bbox.size.x, detect.bbox.size.y, detect.bbox.size.z);
        tf2::Matrix3x3 rot_m(transform.getRotation());
        s = rot_m * s;
        s = s.absolute();
        detect.bbox.size.x = s.x();
        detect.bbox.size.y = s.y();
        detect.bbox.size.z = s.z();
      }

      // for(auto& detect : old_message->detections){
      //   auto bbox = detect.bbox;
      //   std::cout << "bbox AFTER transform values: " 
      //               << " x=" << bbox.center.position.x
      //               << " y=" << bbox.center.position.y
      //               << " z=" << bbox.center.position.z
      //               << " w=" << bbox.size.x
      //               << " d=" << bbox.size.y
      //               << " h=" << bbox.size.z
      //               << std::endl;
      // }

      old_message->header.frame_id = new_frame;
      return;
    }

  private:
    void decode_detections(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message, vector<Object>& objects) {
        auto detections = detections_message->detections;
        objects.resize(detections.size());
        for(auto detection : detections){
          // Decode
          Object obj;
          obj.box = detection.bbox;
          obj.label = BYTETracker::class_to_int[detection.results[0].id];
          obj.prob = detection.results[0].score;

          objects.push_back(obj);
        }
        #ifdef DEBUG
          std::cout << "Number of objects: " << objects.size() << std::endl;
        #endif
    }

    void add_detection(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message)
    {
      #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detections_message->header.frame_id.c_str());
      #endif
      _num_detections++;
      if (_num_detections % 50 == 0)
      {
          RCLCPP_INFO(this->get_logger(), "Processing frame %d (%d fps)", _num_detections, _num_detections * 1000000 / _total_ms);
          //std::cout << "Processing frame " << _num_detections << " (" << _num_detections * 1000000 / _total_ms << " fps)" << std::endl;
      }
		  if (detections_message->detections.empty())
			  return;
    
      // Decode detections and update the tracker
      auto start = chrono::system_clock::now();

      // Move all the detections to a common fixed frame
      if(detections_message->header.frame_id != _fixed_frame){
        change_frame(detections_message, _fixed_frame);
      }

      vector<Object> objects;
      // Put detection3d array inside the object structure
      decode_detections(detections_message, objects);
      // Get the Tracks for the object detected
      vector<STrack> output_stracks = _tracker.update(objects);

      #ifdef DEBUG
        std::cout << "Tracker updated showing " << output_stracks.size() <<
        (output_stracks.size()>1 ? " tracks" : " track") << std::endl;
      #endif

      auto end = chrono::system_clock::now();
      _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();

      auto out_message = vision_msgs::msg::Detection3DArray();
      out_message.header = detections_message->header;
      out_message.header.frame_id = _fixed_frame;
      out_message.detections.resize(output_stracks.size());
      for (unsigned int i = 0; i < output_stracks.size(); i++)
      {
        auto current_track = output_stracks[i];
        vector<float> minwdh = current_track.minwdh;

        #ifdef DEBUG
          std::cout << "Seeing " << current_track.class_name << " number " << current_track.track_id << " -- Score: " << current_track.score << std::endl;
          std::cout << "minwdh values: " 
                    << " x=" << minwdh[0] + minwdh[3]/2
                    << " y=" << minwdh[1] + minwdh[4]/2
                    << " z=" << minwdh[2] + minwdh[5]/2
                    << " w=" << minwdh[3]
                    << " d=" << minwdh[4]
                    << " h=" << minwdh[5]
                    << std::endl;
        #endif
        // bool vertical = tlwh[2] / tlwh[3] > 1.6; <-- show only vertical and big rectangles
        // if (tlwh[2] * tlwh[3] > 20 && !vertical)

        // Show tracking
        vision_msgs::msg::Detection3D single_det = vision_msgs::msg::Detection3D();
        single_det.header = out_message.header;
        single_det.is_tracking = current_track.is_activated;
        single_det.bbox.center.position.x = minwdh[0] + minwdh[3]/2;
        single_det.bbox.center.position.y = minwdh[1] + minwdh[4]/2;
        single_det.bbox.center.position.z = minwdh[2] + minwdh[5]/2;
        single_det.bbox.center.orientation.w = 1.0;
        single_det.bbox.size.x =            minwdh[3];
        single_det.bbox.size.y =            minwdh[4];
        single_det.bbox.size.z =            minwdh[5];

        auto hypothesis = vision_msgs::msg::ObjectHypothesisWithPose();
        hypothesis.id = current_track.class_name; //.append(to_string(current_track.track_id));
        hypothesis.score = current_track.score;
        single_det.results.push_back(hypothesis);

        out_message.detections.push_back(single_det);
        
        // Scalar s_color = _tracker.get_color(output_stracks[i].track_id);
        // putText(img, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5), 
        //                 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
        // rectangle(img, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s_color, 2);

      }

      _det_publisher->publish(out_message);
      // Show Progress
      std::cout << format("frame: %d fps: %d num_tracks: %lu", _num_detections, _num_detections * 1000000 / _total_ms, output_stracks.size()) << "\r";
      std::cout.flush();
    }


  private:

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _det_publisher;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _detection;
    std::string _fixed_frame;
    BYTETracker _tracker;
    int _num_detections;
    int _total_ms;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBTracker>());
  rclcpp::shutdown();
  return 0;
}