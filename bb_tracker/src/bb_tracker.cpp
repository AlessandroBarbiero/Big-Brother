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
        this->declare_parameter("fps", 30);
        int fps = get_parameter("fps").as_int();

        _detection = this->create_subscription<vision_msgs::msg::Detection3DArray>(
                "detection_3d", 10, std::bind(&BBTracker::add_detection, this, _1));

        
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
        // if (box_prob > prob_threshold)
        //     {
        //         Object obj;
        //         obj.rect.x = x0;
        //         obj.rect.y = y0;
        //         obj.rect.width = w;
        //         obj.rect.height = h;
        //         obj.label = class_idx;
        //         obj.prob = box_prob;

        //         objects.push_back(obj);
        //     }
        }
        //std::cout << "num of boxes before nms: " << proposals.size() << std::endl;

        // Sorting
        //qsort_descent_inplace(proposals);
}

    void add_detection(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message)
    {
      RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detections_message->header.frame_id.c_str());

      _num_detections++;
      if (_num_detections % 20 == 0)
      {
          cout << "Processing frame " << _num_detections << " (" << _num_detections * 1000000 / _total_ms << " fps)" << endl;
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
      auto end = chrono::system_clock::now();
      _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();

      for (int i = 0; i < output_stracks.size(); i++)
      {
        vector<float> tlwh = output_stracks[i].tlwh;
        bool vertical = tlwh[2] / tlwh[3] > 1.6;
        if (tlwh[2] * tlwh[3] > 20 && !vertical)
        {
          Scalar s = _tracker.get_color(output_stracks[i].track_id);
          // TODO: Show Tracking
          // putText(img, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5), 
          //                 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
          // rectangle(img, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
        }
      }
      // TODO: Show Progress
      // putText(img, format("frame: %d fps: %d num_tracks: %d", _num_detections, _num_detections * 1000000 / _total_ms, output_stracks.size()), 
      //         Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
    }

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