#include <bb_tracker/bb_tracker.hpp>

// #define DEBUG
#define NO_PROGRESS // Avoid printing the fps and number of tracks
#define OBJECT_BUFFER_SIZE 500

BBTracker::BBTracker()
: Node("bb_tracker"), _tf_buffer(this->get_clock()), _tf_listener(_tf_buffer)
{
  // %%%%%%%%%% ROS Parameters %%%%%%%%%%%%%%
  // Visualization
  auto show_image_projection_desc = rcl_interfaces::msg::ParameterDescriptor{};
  show_image_projection_desc.description = "Set to true to visualize the projection of the objects considered by the tracker and the new detections 2D on the image, the image topic should be remapped to bytetrack/camera_image";
  auto show_covariance_desc = rcl_interfaces::msg::ParameterDescriptor{};
  show_covariance_desc.description = "Set to true to visualize the covariance of the position of the objects considered by the tracker";
  auto points_track_desc = rcl_interfaces::msg::ParameterDescriptor{};
  points_track_desc.description = "Number of points to show per each track.";
  // Detection
  auto left_handed_system_desc = rcl_interfaces::msg::ParameterDescriptor{};
  left_handed_system_desc.description = "Set to true to if the Detections 3D come from a left handed system like Unreal Engine (yaw angle is rotated 180 degrees)";
  auto max_dt_desc = rcl_interfaces::msg::ParameterDescriptor{};
  max_dt_desc.description = "Max interval (current time - new detection time) in milliseconds, older detections are discarded";
  auto camera_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  camera_list_desc.description = "List of the CameraInfo topics";
  auto image_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  image_list_desc.description = "List of the Images retreived from cameras topics, it should have the same order and size as camera_list if show_image_projection is set";
  auto det2d_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  det2d_list_desc.description = "List of the Detection2DArray topics used together with the CameraInfo of camera_list, it should have the same order and size as camera_list";
  auto det3d_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  det3d_list_desc.description = "List of the Detection3DArray topics from which the tracker will read the 3D detections";
  // Tracker Time
  auto time_to_lost_desc = rcl_interfaces::msg::ParameterDescriptor{};
  time_to_lost_desc.description = "Milliseconds a tracked object is not seen to declare it lost.";
  auto unconfirmed_ttl_desc = rcl_interfaces::msg::ParameterDescriptor{};
  unconfirmed_ttl_desc.description = "Milliseconds an unconfirmed object is not seen before removing it.";
  auto lost_ttl_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lost_ttl_desc.description = "Number of milliseconds a track is kept if it is not seen in the scene.";
  // Tracker Thresholds
  auto t_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  t_thresh_desc.description = "This threshold is used to divide initially the detections into high and low score, high score detections are considered first for matching";
  auto h_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  h_thresh_desc.description = "This threshold is used to determine whether a high score detected object should be considered as a new object if it does not match with previously considered tracklets";
  auto m_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  m_thresh_desc.description = "This threshold is used during the association step to establish the correspondence between existing tracks and newly detected objects.";
  // Kalman Filter
  auto mul_p03d_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mul_p03d_desc.description = "List of multiplicators for the varaince P0 in 3D detection to use within the Kalman Filter";
  auto mul_p02d_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mul_p02d_desc.description = "List of multiplicators for the varaince P0 in 2D detection to use within the Kalman Filter";
  auto mul_process_noise_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mul_process_noise_desc.description = "List of multiplicators for the varaince of the Process Noise to use within the Kalman Filter";
  auto mul_mn3d_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mul_mn3d_desc.description = "List of multiplicators for the varaince of the Measurement Noise in 3D detections to use within the Kalman Filter";
  auto mul_mn2d_desc = rcl_interfaces::msg::ParameterDescriptor{};
  mul_mn2d_desc.description = "List of multiplicators for the varaince of the Measurement Noise in 2D detections to use within the Kalman Filter";

  auto fixed_frame_desc = rcl_interfaces::msg::ParameterDescriptor{};
  fixed_frame_desc.description = "The fixed frame the BYTETracker has to use, all the detections has to give a transform for this frame";

  vector<double> mul_p03d(8,1.0), mul_p02d(8,1.0), mul_process_noise(8,1.0), mul_mn3d(6,1.0), mul_mn2d(5,1.0);
  vector<std::string> camera_info_topics = {"/camera/info1"};
  vector<std::string> image_topics = {"/camera/image1"};
  vector<std::string> det2d_topics = {"bytetrack/detections2d"};
  vector<std::string> det3d_topics = {"bytetrack/detections3d"};

  // Visualization
  this->declare_parameter("show_img_projection", false, show_image_projection_desc);
  this->declare_parameter("show_covariance", false, show_covariance_desc);
  this->declare_parameter("points_per_track", 100, points_track_desc);
  // Detection
  this->declare_parameter("left_handed_system", false, left_handed_system_desc);
  this->declare_parameter("max_dt_past", 2000, max_dt_desc);
  this->declare_parameter("camera_list", camera_info_topics, camera_list_desc);
  this->declare_parameter("image_list", image_topics, image_list_desc);
  this->declare_parameter("detection2d_list", det2d_topics, det2d_list_desc);
  this->declare_parameter("detection3d_list", det3d_topics, det3d_list_desc);

  // Tracker Time
  this->declare_parameter("time_to_lost", 300, time_to_lost_desc);
  this->declare_parameter("unconfirmed_ttl", 300, unconfirmed_ttl_desc);
  this->declare_parameter("lost_ttl", 1000, lost_ttl_desc);
  // Tracker Thresholds
  this->declare_parameter("track_thresh", 0.5, t_thresh_desc);
  this->declare_parameter("high_thresh", 0.6, h_thresh_desc);
  this->declare_parameter("match_thresh", 0.8, m_thresh_desc);
  // Kalman Filter
  this->declare_parameter("mul_p03d", mul_p03d, mul_p03d_desc);
  this->declare_parameter("mul_p02d", mul_p02d, mul_p02d_desc);
  this->declare_parameter("mul_process_noise", mul_process_noise, mul_process_noise_desc);
  this->declare_parameter("mul_mn3d", mul_mn3d, mul_mn3d_desc);
  this->declare_parameter("mul_mn2d", mul_mn2d, mul_mn2d_desc);

  this->declare_parameter("fixed_frame", "sensors_home", fixed_frame_desc);

  // Visualization
  int points_per_track=   get_parameter("points_per_track").as_int();
  // Detection
  bool lh_system =        get_parameter("left_handed_system").as_bool();
  int max_dt_past =       get_parameter("max_dt_past").as_int();
  camera_info_topics =    get_parameter("camera_list").as_string_array();
  image_topics =          get_parameter("image_list").as_string_array();
  det2d_topics =          get_parameter("detection2d_list").as_string_array();
  det3d_topics =          get_parameter("detection3d_list").as_string_array();

  // Tracker Time
  int time_to_lost    =   get_parameter("time_to_lost").as_int();
  int unconfirmed_ttl =   get_parameter("unconfirmed_ttl").as_int();
  int lost_ttl        =   get_parameter("lost_ttl").as_int();
  // Tracker Thresholds
  float h_thresh =        get_parameter("high_thresh").as_double();
  float t_thresh =        get_parameter("track_thresh").as_double();
  float m_thresh =        get_parameter("match_thresh").as_double();
  // Kalman Filter
  mul_p03d =              get_parameter("mul_p03d").as_double_array();
  mul_p02d =              get_parameter("mul_p02d").as_double_array();
  mul_process_noise =     get_parameter("mul_process_noise").as_double_array();
  mul_mn3d =              get_parameter("mul_mn3d").as_double_array();
  mul_mn2d =              get_parameter("mul_mn2d").as_double_array();
  
  _fixed_frame =          get_parameter("fixed_frame").as_string();


  // %%%%%%%%% ROS Subscriber and Publisher %%%%%%%%%%
  cout << "Listening Detection 3D from topics:" << endl;
  for(auto det3d_topic : det3d_topics){
    auto detection3d = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            det3d_topic, 10, std::bind(&BBTracker::add_detection3D, this, _1));
    _detections3d.push_back(detection3d);
    
    cout << "\t" << det3d_topic << endl;
  }

  if(get_parameter("show_img_projection").as_bool()){
    int id = 0;
    for (auto camera_topic : camera_info_topics){
      std::function< void
                        ( const vision_msgs::msg::Detection2DArray::ConstSharedPtr&, 
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr&, 
                          const sensor_msgs::msg::Image::ConstSharedPtr&
                        ) 
                    > callback_fun = std::bind(
        &BBTracker::add_detection2D_image, this, id, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      
      _camera_info_subs.emplace_back(std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, camera_topic));
      _camera_image_subs.emplace_back(std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, image_topics[id]));
      _detection2d_subs.emplace_back(std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, det2d_topics[id]));

      std::shared_ptr<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, 
                                                        sensor_msgs::msg::CameraInfo, 
                                                        sensor_msgs::msg::Image>> sync_det_image;
      sync_det_image = std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::CameraInfo, 
          sensor_msgs::msg::Image>>(*_detection2d_subs[id], *_camera_info_subs[id], *_camera_image_subs[id], 100);
      // It needs a double bind, don't know why
      sync_det_image->registerCallback(std::bind(callback_fun, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

      _sync_det_images.push_back(sync_det_image);

      cout << "Listening Detection 2D (" << id << ") using"
        "\n\tCameraInfo topic:\t" << camera_topic << 
        "\n\tImage topic:\t\t" << image_topics[id] <<
        "\n\tDetection topic:\t" << det2d_topics[id] 
        << endl;

      id++;
    }

    // This is the single TimeSync
    // _camera_image.subscribe(this, "bytetrack/camera_image");
    // _sync_det_camera = std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image>>(
    // _detection2d, _camera_info, _camera_image, 100);
    // _sync_det_camera->registerCallback(std::bind(&BBTracker::add_detection2D_image, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));       
  }
  else{
    int id = 0;
    for (auto camera_topic : camera_info_topics){
      std::function< void
                        ( const vision_msgs::msg::Detection2DArray::ConstSharedPtr&, 
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
                        ) 
                    > callback_fun = std::bind(
        &BBTracker::add_detection2D, this, std::placeholders::_1, std::placeholders::_2);
      
      _camera_info_subs.emplace_back(std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, camera_topic));
      _detection2d_subs.emplace_back(std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, det2d_topics[id]));

      std::shared_ptr<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, 
                                                        sensor_msgs::msg::CameraInfo>> sync_det_no_image;
      sync_det_no_image = std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, 
                                                                          sensor_msgs::msg::CameraInfo>>(*_detection2d_subs[id],
                                                                           *_camera_info_subs[id], 100);
      // It needs a double bind, don't know why
      sync_det_no_image->registerCallback(std::bind(callback_fun, std::placeholders::_1, std::placeholders::_2));

      _sync_det_no_images.push_back(sync_det_no_image);

      cout << "Listening Detection 2D (" << id << ") using"
        "\n\tCameraInfo topic:\t" << camera_topic << 
        "\n\tDetection topic:\t\t" << det2d_topics[id] 
        << endl;

      id++;
    }

    // This is for the single TimeSync
    // _sync_det2d = std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::CameraInfo>>(
    // _detection2d, _camera_info, 100);
    // _sync_det2d->registerCallback(std::bind(&BBTracker::add_detection2D, this, std::placeholders::_1, std::placeholders::_2));       
  }

  _det_publisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("bytetrack/active_tracks", 10);
  _det_poses_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("bytetrack/poses", 10);
  _path_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("bytetrack/active_paths", 10);
  _text_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("bytetrack/text", 10);
  _covariance_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("bytetrack/covariance", 10);

  _tracks_publisher = this->create_publisher<bb_interfaces::msg::STrackArray>("bytetrack/active_tracks_explicit", 10);

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // Add callback when a parameter is changed
  _callback_handle = this->add_on_set_parameters_callback(std::bind(&BBTracker::parametersCallback, this, std::placeholders::_1));

  // Reserve space trying to avoid frequent dynamical reallocation
  _objects_buffer.reserve(OBJECT_BUFFER_SIZE);

  STrack::last_points_capacity = points_per_track;

  // Init BYTETracker object
  _tracker.init(time_to_lost, unconfirmed_ttl, lost_ttl, max_dt_past, t_thresh, h_thresh, m_thresh, lh_system);
  _tracker.initVariance(mul_p03d, mul_p02d, mul_process_noise, mul_mn3d, mul_mn2d);
  _num_updates = 0;
  _total_ms = 0;

  RCLCPP_INFO(this->get_logger(), "Ready to track");
}

vector<STrack*> BBTracker::update_tracker(std::vector<Object3D>& new_objects){
  _num_updates++;

  // Get the Tracks for the objects currently beeing tracked
  return _tracker.update(new_objects);
}

vector<STrack*> BBTracker::update_tracker(std::vector<Object2D>& new_objects, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info){
  _num_updates++;

  // Get values for 2D update from camera info
  TRANSFORMATION V = getViewMatrix(_fixed_frame, camera_info->header.frame_id);
  PROJ_MATRIX P;
  int p_index = 0;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
          P(i, j) = camera_info->p[p_index];
          p_index++;
      }
  }

  // Get the Tracks for the objects currently beeing tracked
  return _tracker.update(new_objects, P, V, camera_info->width, camera_info->height);
}

void BBTracker::change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, const std::string& new_frame){
  std::string old_frame = old_message->header.frame_id;
  geometry_msgs::msg::TransformStamped tf_result;
  try {
    tf_result = _tf_buffer.lookupTransform(new_frame, old_frame, rclcpp::Time(0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Change_frame-> No transform exists for the given tfs: " << old_frame << " - " << new_frame);
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

  old_message->header.frame_id = new_frame;
  return;
}

void BBTracker::decode_detections(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message, vector<Object3D>& objects) {
  auto detections = detections_message->detections;
  objects.reserve(detections.size());
  for(auto detection : detections){
    // Decode
    Object3D obj;
    obj.box = detection.bbox;
    obj.label = BYTETracker::class_to_label[detection.results[0].id];
    obj.prob = detection.results[0].score;
    obj.time_ms = detection.header.stamp.sec*1000 + detection.header.stamp.nanosec/1e+6;

    objects.push_back(obj);
  }
  #ifdef DEBUG
    std::cout << "Number of objects 3D: " << objects.size() << std::endl;
  #endif
}

void BBTracker::decode_detections(const std::shared_ptr<const vision_msgs::msg::Detection2DArray> detections_message, vector<Object2D>& objects) {
    auto detections = detections_message->detections;
    objects.reserve(detections.size());
    for(auto detection : detections){
      // Decode
      Object2D obj;
      // if(detection.bbox.center.theta != 0){
      //   //TODO: think about handling tilted bbox
      // }
      obj.tlbr = {
        static_cast<float>(detection.bbox.center.x-detection.bbox.size_x/2),
        static_cast<float>(detection.bbox.center.y-detection.bbox.size_y/2),
        static_cast<float>(detection.bbox.center.x+detection.bbox.size_x/2),
        static_cast<float>(detection.bbox.center.y+detection.bbox.size_y/2),
         };
      obj.label = BYTETracker::class_to_label[detection.results[0].id];
      obj.prob = detection.results[0].score;
      obj.time_ms = detection.header.stamp.sec*1000 + detection.header.stamp.nanosec/1e+6;

      objects.push_back(obj);
    }
    #ifdef DEBUG
      std::cout << "Number of objects 2D: " << objects.size() << std::endl;
    #endif
}

// %%%%%%%%%% Callbacks

void BBTracker::add_detection3D(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message)
{
  #ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detections_message->header.frame_id.c_str());
  #endif

  if (detections_message->detections.empty())
    return;

  auto start = chrono::system_clock::now();

// -----------------------

  // Move all the detections to a common fixed frame
  if(detections_message->header.frame_id != _fixed_frame){
    change_frame(detections_message, _fixed_frame);
  }

  vector<Object3D> objects3D;
  // Put detection3d array inside the object structure
  decode_detections(detections_message, objects3D);
  vector<STrack*> output_stracks = update_tracker(objects3D);
  publish_stracks(output_stracks);

// ----------------------

  auto end = chrono::system_clock::now();
  auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
  _total_ms = _total_ms + duration;

  #ifndef NO_PROGRESS
    std::cout << format("(3D) update: %d fps: %d num_tracks: %lu", _num_updates, static_cast<int>(MICRO_IN_SECOND / duration), output_stracks.size()) << "\r";
    std::cout.flush();
  #endif

  if (_num_updates % 50 == 0)
  {
      RCLCPP_INFO(this->get_logger(), "Update number %d, (Average %d fps | Current %d fps)", _num_updates, static_cast<int>(_num_updates * MICRO_IN_SECOND / _total_ms), static_cast<int>(MICRO_IN_SECOND / duration));
  }
}

void BBTracker::add_detection2D(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info)
{
  #ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detection_msg->header.frame_id.c_str());
  #endif

  if (detection_msg->detections.empty())
    return;

  auto start = chrono::system_clock::now();

  // ------------------------

  vector<Object2D> objects2D;
  // Put detection2d array inside the object structure
  decode_detections(detection_msg, objects2D);
  vector<STrack*> output_stracks = update_tracker(objects2D, camera_info);
  publish_stracks(output_stracks);

  // ------------------------

  auto end = chrono::system_clock::now();
  auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
  _total_ms = _total_ms + duration;

  #ifndef NO_PROGRESS
   std::cout << format("(2D) update: %d fps: %d num_tracks: %lu", _num_updates, static_cast<int>(MICRO_IN_SECOND / duration), output_stracks.size()) << "\r";
   std::cout.flush();
  #endif

  if (_num_updates % 50 == 0)
  {
      RCLCPP_INFO(this->get_logger(), "Update number %d, (Average %d fps | Current %d fps)", _num_updates, static_cast<int>(_num_updates * MICRO_IN_SECOND / _total_ms), static_cast<int>(MICRO_IN_SECOND / duration));
  }
}

rcl_interfaces::msg::SetParametersResult BBTracker::parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for(auto p : parameters){
    if(p.get_name() == "show_img_projection" && p.get_type_name() == "bool" && p.as_bool() == false){
      cv::destroyAllWindows();
    }
  }
  return result;
}

// TODO: delete
// Function that prints the posizion of the mouse on click over image
// void mouse_callback(int event, int x, int y, int flags, void* userdata)
// {
//   if  ( event == EVENT_LBUTTONDOWN )
//   {
//     cout << "----> Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//   }
// }

// Add a color legend on the right showing colors and relative label on the image
void addColorLegend(cv::Mat& image, const std::vector<cv::Scalar>& colors, const std::vector<std::string>& labels) {
    int legendWidth = 150;  // Width of the legend box
    int legendHeight = 5 + 20 * colors.size();  // Height of the legend box, with a little space between each color
    int startX = image.cols - legendWidth;  // X-coordinate to start drawing the legend box
    int startY = 10;  // Y-coordinate to start drawing the legend box at the top

    // Create a white background for the legend
    cv::Rect legendRect(startX, startY, legendWidth, legendHeight);
    cv::rectangle(image, legendRect, cv::Scalar(255, 255, 255), -1);

    // Draw rectangles with colors and corresponding labels in the legend
    int rectWidth = 15;  // Width of the color square
    int rectHeight = 15;  // Height of the color square
    for (size_t i = 0; i < colors.size(); ++i) {
        cv::Rect rect(startX + 5, startY + 5 + i * (rectHeight + 5), rectWidth, rectHeight);
        cv::rectangle(image, rect, colors[i], -1);
        cv::putText(image, labels[i], cv::Point(startX + rectWidth + 15, startY + 5 + i * (rectHeight + 5) + rectHeight - 3),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    }
}

void BBTracker::add_detection2D_image(int id, const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg, 
                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info, 
                                      const sensor_msgs::msg::Image::ConstSharedPtr& image) {

  add_detection2D(detection_msg, camera_info);

  // Check that the parameter is still set, if not avoid publishing the projections
  if(!get_parameter("show_img_projection").as_bool()){
    return;
  }

  cv_bridge::CvImagePtr cv_ptr_track;

  std::vector<STrack> trackedObj = this->_tracker.getTrackedObj();

  // Predict the position of the objects before drawing them
  std::vector<STrack *> objPtr;
  for(size_t i=0; i<trackedObj.size(); i++){
    objPtr.push_back(&trackedObj[i]);
  }
  int64_t current_time_ms = detection_msg->header.stamp.sec*MILLIS_IN_SECONDS + detection_msg->header.stamp.nanosec/NANO_IN_MILLIS;
  //cout << "Predict at time of 2d detection for display on image" << endl;
  STrack::multi_predict(objPtr, _tracker.kalman_filter, current_time_ms);

  // TODO: delete Fake points to test the draw
  // std::vector<STrack> trackedObj;
  // trackedObj.push_back(real_trackedObj[0]);
  // trackedObj[0].minwdh = {-30,15,0,2,2,2};

  // ---------

  TRANSFORMATION vMat = getViewMatrix(_fixed_frame, camera_info->header.frame_id);

  // Projection matrix in this way is the same as using the one of image_geometry::PinholeCameraModel 
  // and is the same as a simple K matrix because it is not adding any rotation and translation respect to the camera frame
  // P = [K | 0]
  PROJ_MATRIX P;
  int p_index = 0;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
          P(i, j) = camera_info->p[p_index];
          p_index++;
      }
  }

  try
  {
    cv_ptr_track = cv_bridge::toCvCopy(image, image->encoding);
    // Draw ellipses for tracked objects
    for(auto obj :trackedObj)
    {
      // show only activated stracks
      #ifndef SHOW_INACTIVE_TRACKS
      if(obj.is_activated)
      #endif
        draw_ellipse(cv_ptr_track, obj, P, vMat);
    }

    // Draw ellipses for incoming detections
    for(auto det : detection_msg->detections){
      cv::ellipse(cv_ptr_track->image, Point(det.bbox.center.x, det.bbox.center.y),
              Size(det.bbox.size_x/2.0, det.bbox.size_y/2.0), 0, 0,
              360, CV_RGB(255, 0, 0),
              1, LINE_AA);
      // If the score is enough to initiate a new object write it in green
      cv::Scalar color_text = (det.results[0].score > _tracker.high_thresh ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0));
      cv::putText(cv_ptr_track->image, format("%.2f", det.results[0].score), Point(det.bbox.center.x, det.bbox.center.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, color_text, 1);
    }

    addColorLegend(cv_ptr_track->image, {CV_RGB(0, 0, 255), CV_RGB(255, 0, 0)}, {"Tracked Objects", "Detected Objects"});

    std::string window_name1 = format("Tracked and Detected Objects (%d)", id);
    cv::imshow(window_name1, cv_ptr_track->image);
    // cv::setMouseCallback(window_name, mouse_callback);
    cv::waitKey(1);

  }
  catch (cv_bridge::Exception& e)
  {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }

  return;
}

void BBTracker::test_ellipse_project(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info, const sensor_msgs::msg::Image::ConstSharedPtr& image){
  cv_bridge::CvImagePtr cv_ptr;
  std::vector<STrack> trackedObj = this->_tracker.getTrackedObj();
  std::vector<STrack *> objPtr;
  for(size_t i=0; i<trackedObj.size(); i++){
    objPtr.push_back(&trackedObj[i]);
  }
  // Predict the position of the objects before drawing them
  int64_t current_time = camera_info->header.stamp.sec*1000 + camera_info->header.stamp.nanosec/1e+6;
  //cout << "Test ellipse project prediction" << endl;
  STrack::multi_predict(objPtr, _tracker.kalman_filter, current_time);

  TRANSFORMATION vMat = getViewMatrix(_fixed_frame, camera_info->header.frame_id);

  // Projection matrix in this way is the same as using the one of image_geometry::PinholeCameraModel 
  // and is the same as a simple K matrix because it is not adding any rotation and translation respect to the camera frame
  // P = [K | 0]
  PROJ_MATRIX P;
  int p_index = 0;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
          P(i, j) = camera_info->p[p_index];
          p_index++;
      }
  }

  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
    // Draw ellipses for tracked objects
    for(auto obj :trackedObj)
      draw_ellipse(cv_ptr, obj, P, vMat);

    std::string window_name = "Image Window";
    cv::imshow(window_name, cv_ptr->image);
    cv::waitKey(1);

  }
  catch (cv_bridge::Exception& e)
  {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
  return;
}

TRANSFORMATION BBTracker::getViewMatrix(const std::string& from_tf, const std::string& camera_tf){
  TRANSFORMATION vMat;

  geometry_msgs::msg::TransformStamped tf_result;
  try {
    tf_result = _tf_buffer.lookupTransform(camera_tf, from_tf, rclcpp::Time(0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_INFO_STREAM(this->get_logger(), "getViewMatrix-> No transform exists for the given tfs: " << from_tf << " - " << camera_tf);
    return vMat.setZero();
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

  tf2::Matrix3x3 rot_m(q);
  
  // vMat = [Rot  | center_of_other_tf_in_new_tf]
  //        [0    |               1             ]
  vMat << rot_m[0][0], rot_m[0][1], rot_m[0][2], p.x(),
          rot_m[1][0], rot_m[1][1], rot_m[1][2], p.y(),
          rot_m[2][0], rot_m[2][1], rot_m[2][2], p.z(),
          0.0,          0.0,          0.0,       1.0;  

  // Same as doing these operations to find view Matrix starting from the inverse transform
  // TRANSFORMATION cameraMat, vMat , rotMat, transMat;
  // transMat << 1.0, 0.0, 0.0, p.x(),
  //             0.0, 1.0, 0.0, p.y(),
  //             0.0, 0.0, 1.0, p.z(),
  //             0.0, 0.0, 0.0, 1.0;
  // rotMat << rot_m[0][0], rot_m[0][1], rot_m[0][2], 0.0,
  //           rot_m[1][0], rot_m[1][1], rot_m[1][2], 0.0,
  //           rot_m[2][0], rot_m[2][1], rot_m[2][2], 0.0,
  //           0.0,          0.0,          0.0,       1.0;
  // cameraMat = transMat * rotMat;     
  // vMat = cameraMat.inverse();  

  return vMat;
}

void BBTracker::draw_ellipse(cv_bridge::CvImagePtr image_ptr, STrack obj, PROJ_MATRIX& P, TRANSFORMATION& vMat){
  // Took inspiration from "Factorization based structure from motion with object priors" by Paul Gay et al.
  float cx,cy,cz;

  KAL_MEAN state = obj.state_predicted.mean;

  cx = state(0);
  cy = state(1);
  cz = state(5)/2.0;

  // Filter out objects behind the camera
  if(objectBehindCamera(cx,cy,cz,vMat))
    return;

  ELLIPSE_STATE ellipse_state = ellipseFromEllipsoidv2(state, vMat, P);

  float ea, eb, ecx, ecy, theta; // Variables of the ellipse

  ecx = ellipse_state(0);
  ecy = ellipse_state(1);
  ea = ellipse_state(2);
  eb = ellipse_state(3);
  theta = ellipse_state(4);

  if(ellipseInImage(ecx,ecy,ea,eb,  image_ptr->image.cols,image_ptr->image.rows)){
    // Draw the rectangle around the ellipse
    cv::Point2i min_pt, max_pt;
    vector<float> tlbr_ellipse = tlbrFromEllipse(ecx,ecy,ea,eb,theta);
    min_pt = {static_cast<int>(tlbr_ellipse[0]), static_cast<int>(tlbr_ellipse[1])};
    max_pt = {static_cast<int>(tlbr_ellipse[2]), static_cast<int>(tlbr_ellipse[3])};
    cv::rectangle(image_ptr->image, cv::Rect(min_pt, max_pt), CV_RGB(0,0,255), 1, LINE_AA);

    // Draw the ellipse
    theta = theta * 180/M_PI; // Convert in degrees
    cv::ellipse(image_ptr->image, Point(ecx, ecy),
                Size(ea, eb), theta, 0,
                360, CV_RGB(0, 0, 255),
                1, LINE_AA);
  }
}

// %%%%%%%%%% Visualization

visualization_msgs::msg::Marker drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix, int32_t id, std_msgs::msg::Header header)
{
  visualization_msgs::msg::Marker tempMarker;
  tempMarker.header.frame_id = header.frame_id;
  tempMarker.header.stamp = header.stamp;
  tempMarker.type = visualization_msgs::msg::Marker::CYLINDER;
  tempMarker.action = visualization_msgs::msg::Marker::ADD;
  tempMarker.ns = "Covariance2D";
  tempMarker.lifetime = rclcpp::Duration(0,200 * NANO_IN_MILLIS);

  tempMarker.id = id;
  tempMarker.pose.position.x = mean[0];
  tempMarker.pose.position.y = mean[1];
  tempMarker.pose.position.z = 0;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

  const Eigen::Vector2f& eigValues (eig.eigenvalues());
  const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

  float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

  double lengthMajor = sqrt(eigValues[0]);
  double lengthMinor = sqrt(eigValues[1]);

  tempMarker.scale.x = 10 * lengthMajor;
  tempMarker.scale.y = 10 * lengthMinor;
  tempMarker.scale.z = 0.001;

  tempMarker.pose.orientation.w = cos(angle*0.5);
  tempMarker.pose.orientation.z = sin(angle*0.5);

  // Make it purple
  tempMarker.color.r = 128;
  tempMarker.color.g = 0;
  tempMarker.color.b = 128;
  tempMarker.color.a = 0.5;

  return tempMarker;
}

void BBTracker::publish_stracks(vector<STrack*>& output_stracks){
  vision_msgs::msg::Detection3DArray out_message;
  geometry_msgs::msg::PoseArray poses_message;
  visualization_msgs::msg::MarkerArray path_markers;
  visualization_msgs::msg::MarkerArray text_markers;
  visualization_msgs::msg::MarkerArray covariance_markers;
  bb_interfaces::msg::STrackArray s_track_array_msg;

  convert_into_detections(output_stracks, &out_message);

  bool show_covariance = get_parameter("show_covariance").as_bool();

  if(show_covariance){
    covariance_markers.markers.clear();
    covariance_markers.markers.reserve(output_stracks.size());
  }

  poses_message.header = out_message.header;
  s_track_array_msg.header = out_message.header;

  path_markers.markers.clear();
  text_markers.markers.clear();
  path_markers.markers.reserve(output_stracks.size());
  text_markers.markers.reserve(output_stracks.size());
  poses_message.poses.reserve(output_stracks.size());
  s_track_array_msg.tracks.reserve(output_stracks.size());
  for (unsigned int i = 0; i < output_stracks.size(); i++)
  {
    auto current_track = output_stracks[i];
    vector<float> minwdh = current_track->minwdh;

    auto single_det = out_message.detections[i];

    s_track_array_msg.tracks.push_back(current_track->toMessage());
    // Publish a path for each track, with relative text
    visualization_msgs::msg::Marker text;
    visualization_msgs::msg::Marker path_marker = createPathMarker(current_track, out_message.header, single_det.bbox.center.position, text);
    text.pose.position.z += single_det.bbox.size.z/2;
    path_markers.markers.push_back(path_marker);
    text_markers.markers.push_back(text);
    poses_message.poses.push_back(single_det.bbox.center);

    if(show_covariance)
      covariance_markers.markers.push_back(drawCovariance( current_track->state_current.mean.block<1, 2>(0, 0),
                                                         current_track->state_current.covariance.block<2, 2>(0, 0), 
                                                         current_track->track_id, out_message.header));
  }

  _det_publisher->publish(out_message);
  _tracks_publisher->publish(s_track_array_msg);
  _det_poses_publisher->publish(poses_message);
  _path_publisher->publish(path_markers);
  _text_publisher->publish(text_markers);

  if(show_covariance)
    _covariance_publisher->publish(covariance_markers);

}

void BBTracker::convert_into_detections(vector<STrack*>& in_stracks, vision_msgs::msg::Detection3DArray* out_message){
  out_message->header.stamp.sec = static_cast<uint32_t>(_tracker.current_time_ms/MILLIS_IN_SECONDS);
  out_message->header.stamp.nanosec = static_cast<uint32_t>(_tracker.current_time_ms*NANO_IN_MILLIS);
  // out_message->header.stamp = get_clock()->now();
  out_message->header.frame_id = _fixed_frame;
  out_message->detections.reserve(in_stracks.size());
  for (unsigned int i = 0; i < in_stracks.size(); i++)
  {
    auto current_track = in_stracks[i];
    vector<float> minwdh = current_track->minwdh;

    // Show tracking
    vision_msgs::msg::Detection3D single_det = vision_msgs::msg::Detection3D();
    single_det.header = out_message->header;
    single_det.is_tracking = current_track->is_activated;
    single_det.tracking_id = current_track->track_id;
    single_det.bbox.center.position.x = minwdh[0] + minwdh[3]/2;
    single_det.bbox.center.position.y = minwdh[1] + minwdh[4]/2;
    single_det.bbox.center.position.z = minwdh[2] + minwdh[5]/2;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, current_track->theta);
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf);
    single_det.bbox.center.orientation = quat_msg;
    single_det.bbox.size.x =            minwdh[3];
    single_det.bbox.size.y =            minwdh[4];
    single_det.bbox.size.z =            minwdh[5];

    auto hypothesis = vision_msgs::msg::ObjectHypothesisWithPose();
    hypothesis.id = classLabelString[(int)current_track->state_current.label];
    hypothesis.score = current_track->state_current.confidence;
    single_det.results.push_back(hypothesis);

    out_message->detections.push_back(single_det);
  }
  return;
}

void initPathMarker(visualization_msgs::msg::Marker& path_marker, Scalar& color){
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.ns = "TrackPath";
  path_marker.color.r = color[2];
  path_marker.color.g = color[1];
  path_marker.color.b = color[0];
  path_marker.color.a = 1.0;
  path_marker.pose.position.x = 0;
  path_marker.pose.position.y = 0;
  path_marker.pose.position.z = 0;
  path_marker.pose.orientation.x = 0;
  path_marker.pose.orientation.y = 0;
  path_marker.pose.orientation.z = 0;
  path_marker.pose.orientation.w = 1;
  path_marker.scale.x = 0.1;
  path_marker.scale.y = 0.0;
  path_marker.scale.z = 0.0;
  path_marker.lifetime = rclcpp::Duration(1,0);
}

void initTextMarker(visualization_msgs::msg::Marker& text_marker, Scalar& color){
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.ns = "TrackID";
  text_marker.color.r = color[2];
  text_marker.color.g = color[1];
  text_marker.color.b = color[0];
  text_marker.color.a = 1.0;
  text_marker.scale.x = 0.0;
  text_marker.scale.y = 0.0;
  text_marker.scale.z = 1;
  text_marker.lifetime = rclcpp::Duration(1,0);
}

visualization_msgs::msg::Marker BBTracker::createPathMarker(STrack* track, std_msgs::msg::Header& header, geometry_msgs::msg::Point& last_point, visualization_msgs::msg::Marker& text){
  track->path_marker.header = header;
  track->text_marker.header = header;
  
  // Init marker if first visualization
  if(track->path_marker.points.size() == 0){
    track->path_marker.id = track->track_id;
    track->text_marker.id = track->track_id;
    Scalar s_color = _tracker.get_color(track->track_id);
    initPathMarker(track->path_marker, s_color);
    initTextMarker(track->text_marker, s_color);
    track->text_marker.text = classLabelString[(int)track->state_current.label];
    track->text_marker.text.append("-").append(::to_string(track->track_id));
  }
  track->last_points.push_back(last_point);
  track->path_marker.points = track->last_points.buffer_;
  track->text_marker.pose.position = last_point;
  text = track->text_marker;
  return track->path_marker;
}


// %%%%%%%%%%%%%%
// %%%  MAIN  %%%
// %%%%%%%%%%%%%%

int main(int argc, char * argv[])
{
  try{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBTracker>());
    rclcpp::shutdown();
  }catch (...){
    getchar();
  }
  getchar();
  return 0;
}