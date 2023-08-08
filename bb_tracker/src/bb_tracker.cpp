#include <bb_tracker/bb_tracker.hpp>

// #define DEBUG
#define OBJECT_BUFFER_SIZE 500

BBTracker::BBTracker()
: Node("bb_tracker"), _tf_buffer(this->get_clock()), _tf_listener(_tf_buffer)
{
  // ROS Parameters
  auto fps_desc = rcl_interfaces::msg::ParameterDescriptor{};
  fps_desc.description = "Frequency of the Tracker, number of executions per second";
  auto t_buffer_desc = rcl_interfaces::msg::ParameterDescriptor{};
  t_buffer_desc.description = "Number of frames a track is kept if it is not seen in the scene";
  auto t_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  t_thresh_desc.description = "This threshold is used to divide initially the detections into high and low score, high score detections are considered first for matching";
  auto h_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  h_thresh_desc.description = "This threshold is used to determine whether a high score detected object should be considered as a new object if it does not match with previously considered tracklets";
  auto m_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  m_thresh_desc.description = "This threshold is used during the association step to establish the correspondence between existing tracks and newly detected objects.";
  auto fixed_frame_desc = rcl_interfaces::msg::ParameterDescriptor{};
  fixed_frame_desc.description = "The fixed frame the BYTETracker has to use, all the detections has to give a transform for this frame";
  this->declare_parameter("fps", 30, fps_desc);
  this->declare_parameter("track_buffer", 30, t_buffer_desc);
  this->declare_parameter("track_thresh", 0.5, t_thresh_desc);
  this->declare_parameter("high_thresh", 0.6, h_thresh_desc);
  this->declare_parameter("match_thresh", 0.8, m_thresh_desc);
  this->declare_parameter("fixed_frame", "sensors_home", fixed_frame_desc);

  int fps =         get_parameter("fps").as_int();
  int t_buffer =    get_parameter("track_buffer").as_int();
  float t_thresh =  get_parameter("high_thresh").as_double();
  float h_thresh =  get_parameter("track_thresh").as_double();
  float m_thresh =  get_parameter("match_thresh").as_double();
  _fixed_frame =    get_parameter("fixed_frame").as_string();

  // ROS Subscriber and Publisher
  _detection = this->create_subscription<vision_msgs::msg::Detection3DArray>(
          "bytetrack/detections", 10, std::bind(&BBTracker::add_detection, this, _1));

  _det_publisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("bytetrack/active_tracks", 10);

  // ROS Timer
  std::chrono::milliseconds ms_to_call((1000/fps));
  _timer = this->create_wall_timer(
      ms_to_call, std::bind(&BBTracker::periodic_update, this));

  // Reserve space trying to avoid frequent dynamical reallocation
  _objects_buffer.reserve(OBJECT_BUFFER_SIZE);

  // Init BYTETracker object
  _tracker.init(fps, t_buffer, t_thresh, h_thresh, m_thresh);
  _num_detections = 0;
  _num_updates = 0;
  _total_ms = 0;

  RCLCPP_INFO(this->get_logger(), "Ready to track");
}

void BBTracker::periodic_update(){
  _num_updates++;

  // Update the tracker
  auto start = chrono::system_clock::now();
  // Get the Tracks for the objects currently beeing tracked
  vector<STrack> output_stracks = _tracker.update(_objects_buffer);
  _objects_buffer.clear();

  auto end = chrono::system_clock::now();
  _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();

  #ifdef DEBUG
    std::cout << "Tracker updated showing " << output_stracks.size() <<
    (output_stracks.size()>1 ? " tracks" : " track") << std::endl;
  #endif

  publish_stracks(output_stracks);
  // Show Progress
  std::cout << format("frame: %d fps: %lu num_tracks: %lu", _num_updates, _num_updates * 1000000 / _total_ms, output_stracks.size()) << "\r";
  std::cout.flush();
}

void BBTracker::change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, std::string& new_frame){
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

void BBTracker::decode_detections(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message, vector<Object>& objects) {
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

void BBTracker::add_detection(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message)
{
  #ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detections_message->header.frame_id.c_str());
  #endif
  _num_detections++;
  if (_num_detections % 50 == 0)
  {
      RCLCPP_INFO(this->get_logger(), "Processing Detection number %d, Update number %d (Ideally %lu fps)", _num_detections, _num_updates, _num_updates * 1000000 / _total_ms);
      //std::cout << "Processing frame " << _num_detections << " (" << _num_detections * 1000000 / _total_ms << " fps)" << std::endl;
  }
  if (detections_message->detections.empty())
    return;

  // Decode detections
  auto start = chrono::system_clock::now();

  // Move all the detections to a common fixed frame
  if(detections_message->header.frame_id != _fixed_frame){
    change_frame(detections_message, _fixed_frame);
  }

  vector<Object> objects;
  // Put detection3d array inside the object structure
  decode_detections(detections_message, objects);
  _objects_buffer.insert(_objects_buffer.end(), objects.begin(), objects.end());

  auto end = chrono::system_clock::now();
  _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();
}

void BBTracker::publish_stracks(vector<STrack> output_stracks){
  auto out_message = vision_msgs::msg::Detection3DArray();
  out_message.header.stamp = get_clock()->now();
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

    // Show tracking
    vision_msgs::msg::Detection3D single_det = vision_msgs::msg::Detection3D();
    single_det.header = out_message.header;
    single_det.is_tracking = current_track.is_activated;
    single_det.bbox.center.position.x = minwdh[0] + minwdh[3]/2;
    single_det.bbox.center.position.y = minwdh[1] + minwdh[4]/2;
    single_det.bbox.center.position.z = minwdh[2] + minwdh[5]/2;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, current_track.theta);
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf);
    single_det.bbox.center.orientation = quat_msg;
    single_det.bbox.size.x =            minwdh[3];
    single_det.bbox.size.y =            minwdh[4];
    single_det.bbox.size.z =            minwdh[5];

    auto hypothesis = vision_msgs::msg::ObjectHypothesisWithPose();
    hypothesis.id = current_track.class_name; //.append(to_string(current_track.track_id));
    hypothesis.score = current_track.score;
    single_det.results.push_back(hypothesis);

    out_message.detections.push_back(single_det);
    
    // Example way to get colors for tracks
    // Scalar s_color = _tracker.get_color(output_stracks[i].track_id);
  }

  _det_publisher->publish(out_message);
}



// %%%%%%%%%%%%%%
// %%%  MAIN  %%%
// %%%%%%%%%%%%%%

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBTracker>());
  rclcpp::shutdown();
  return 0;
}