#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <opencv2/opencv.hpp>
#include <bb_tracker/BYTETracker.h>

using std::placeholders::_1;

// #define DEBUG

// Node that tracks the bounding boxes 3D published in a topic using Byte track

class BBTracker : public rclcpp::Node
{
  public:
    BBTracker()
    : Node("bb_tracker")
    {
        this->declare_parameter("fps", 30);
        int fps = get_parameter("fps").as_int();

        _detection = this->create_subscription<vision_msgs::msg::Detection3DArray>(
                "detection_3d", 10, std::bind(&BBTracker::add_detection, this, _1));

        _det_publisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("bytetrack/detections", 10);

        _tracker = BYTETracker(fps, 30);
        _num_detections = 0;
        _total_ms = 0;
    
        RCLCPP_INFO(this->get_logger(), "Ready to track");
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
      if (_num_detections % 20 == 0)
      {
          RCLCPP_INFO(this->get_logger(), "Processing frame %d (%d fps)", _num_detections, _num_detections * 1000000 / _total_ms);
          //std::cout << "Processing frame " << _num_detections << " (" << _num_detections * 1000000 / _total_ms << " fps)" << std::endl;
      }
		  if (detections_message->detections.empty())
			  return;
    
      // Decode detections and update the tracker
      auto start = chrono::system_clock::now();
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
      // TODO: Show Progress
      // putText(img, format("frame: %d fps: %d num_tracks: %d", _num_detections, _num_detections * 1000000 / _total_ms, output_stracks.size()), 
      //         Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
    }


  private:

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _det_publisher;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _detection;
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