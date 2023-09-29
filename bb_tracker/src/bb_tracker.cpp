#include <bb_tracker/bb_tracker.hpp>

// #define DEBUG
#define OBJECT_BUFFER_SIZE 500

BBTracker::BBTracker()
: Node("bb_tracker"), _tf_buffer(this->get_clock()), _tf_listener(_tf_buffer)
{
  // ROS Parameters
  auto time_to_lost_desc = rcl_interfaces::msg::ParameterDescriptor{};
  time_to_lost_desc.description = "Milliseconds a tracked object is not seen to declare it lost.";
  auto unconfirmed_ttl_desc = rcl_interfaces::msg::ParameterDescriptor{};
  unconfirmed_ttl_desc.description = "Milliseconds an unconfirmed object is not seen before removing it.";
  auto lost_ttl_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lost_ttl_desc.description = "Number of milliseconds a track is kept if it is not seen in the scene.";
  auto t_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  t_thresh_desc.description = "This threshold is used to divide initially the detections into high and low score, high score detections are considered first for matching";
  auto h_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  h_thresh_desc.description = "This threshold is used to determine whether a high score detected object should be considered as a new object if it does not match with previously considered tracklets";
  auto m_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  m_thresh_desc.description = "This threshold is used during the association step to establish the correspondence between existing tracks and newly detected objects.";
  auto fixed_frame_desc = rcl_interfaces::msg::ParameterDescriptor{};
  fixed_frame_desc.description = "The fixed frame the BYTETracker has to use, all the detections has to give a transform for this frame";
  this->declare_parameter("time_to_lost", 300, time_to_lost_desc);
  this->declare_parameter("unconfirmed_ttl", 300, unconfirmed_ttl_desc);
  this->declare_parameter("lost_ttl", 1000, lost_ttl_desc);
  this->declare_parameter("track_thresh", 0.5, t_thresh_desc);
  this->declare_parameter("high_thresh", 0.6, h_thresh_desc);
  this->declare_parameter("match_thresh", 0.8, m_thresh_desc);
  this->declare_parameter("fixed_frame", "sensors_home", fixed_frame_desc);

  int time_to_lost    =   get_parameter("time_to_lost").as_int();
  int unconfirmed_ttl =   get_parameter("unconfirmed_ttl").as_int();
  int lost_ttl        =   get_parameter("lost_ttl").as_int();
  float h_thresh =        get_parameter("high_thresh").as_double();
  float t_thresh =        get_parameter("track_thresh").as_double();
  float m_thresh =        get_parameter("match_thresh").as_double();
  _fixed_frame =          get_parameter("fixed_frame").as_string();

  // ROS Subscriber and Publisher
  _detection3d = this->create_subscription<vision_msgs::msg::Detection3DArray>(
          "bytetrack/detections3d", 10, std::bind(&BBTracker::add_detection3D, this, _1));

  _detection2d.subscribe(this, "bytetrack/detections2d");
  _camera_info.subscribe(this, "bytetrack/camera_info");
  _camera_image.subscribe(this, "bytetrack/camera_image");

  _sync_det_camera = std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image>>(
  _detection2d, _camera_info, _camera_image, 100);

  // Register the callback function
  _sync_det_camera->registerCallback(std::bind(&BBTracker::add_detection2D_image, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));       

  // _detection2d = this->create_subscription<vision_msgs::msg::Detection2DArray>(
  //         "bytetrack/detections2d", 10, std::bind(&BBTracker::add_detection2D, this, _1));

  _det_publisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("bytetrack/active_tracks", 10);
  _det_poses_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("bytetrack/poses", 10);
  _path_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("bytetrack/active_paths", 10);
  _text_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("bytetrack/text", 10);

  // ROS Timer [Old version with periodic updates]
  // std::chrono::milliseconds ms_to_call((1000/fps));
  // _timer = this->create_wall_timer(
  //     ms_to_call, std::bind(&BBTracker::update_tracker, this));

  // Reserve space trying to avoid frequent dynamical reallocation
  _objects_buffer.reserve(OBJECT_BUFFER_SIZE);

  // Init BYTETracker object
  _tracker.init(time_to_lost, unconfirmed_ttl, lost_ttl, t_thresh, h_thresh, m_thresh);
  _num_detections = 0;
  _num_updates = 0;
  _total_ms = 0;

  RCLCPP_INFO(this->get_logger(), "Ready to track");
}

void BBTracker::update_tracker(std::vector<Object3D>& new_objects){
  _num_updates++;

  // Update the tracker
  auto start = chrono::system_clock::now();

  // Get the Tracks for the objects currently beeing tracked
  vector<STrack*> output_stracks = _tracker.update(new_objects);

  auto end = chrono::system_clock::now();
  _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();

  #ifdef DEBUG
    std::cout << "Tracker updated showing " << output_stracks.size() <<
    (output_stracks.size()>1 ? " tracks" : " track") << std::endl;
  #endif

  publish_stracks(output_stracks);

  // Show Progress
  #ifndef DEBUG
  std::cout << format("frame: %d fps: %lu num_tracks: %lu", _num_updates, _num_updates * 1000000 / _total_ms, output_stracks.size()) << "\r";
  std::cout.flush();
  #endif

  #ifdef DEBUG
  std::cout<< "Tracks published" << std::endl;
  #endif
}

void BBTracker::update_tracker(std::vector<Object2D>& new_objects){
  _num_updates++;

  // Update the tracker
  auto start = chrono::system_clock::now();

  // Get the Tracks for the objects currently beeing tracked
  vector<STrack*> output_stracks = _tracker.update(new_objects);

  auto end = chrono::system_clock::now();
  _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();

  #ifdef DEBUG
    std::cout << "Tracker updated showing " << output_stracks.size() <<
    (output_stracks.size()>1 ? " tracks" : " track") << std::endl;
  #endif

  publish_stracks(output_stracks);

  // Show Progress
  #ifndef DEBUG
  std::cout << format("frame: %d fps: %lu num_tracks: %lu", _num_updates, _num_updates * 1000000 / _total_ms, output_stracks.size()) << "\r";
  std::cout.flush();
  #endif

  #ifdef DEBUG
  std::cout<< "Tracks published" << std::endl;
  #endif
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

void BBTracker::decode_detections(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message, vector<Object3D>& objects) {
    auto detections = detections_message->detections;
    objects.reserve(detections.size());
    for(auto detection : detections){
      // Decode
      Object3D obj;
      obj.box = detection.bbox;
      obj.label = BYTETracker::class_to_int[detection.results[0].id];
      obj.prob = detection.results[0].score;
      obj.time_ms = detection.header.stamp.sec*1000 + detection.header.stamp.nanosec/1e+6;

      objects.push_back(obj);
    }
    #ifdef DEBUG
      std::cout << "Number of objects: " << objects.size() << std::endl;
    #endif
}

void BBTracker::decode_detections(std::shared_ptr<vision_msgs::msg::Detection2DArray> detections_message, vector<Object2D>& objects) {
    auto detections = detections_message->detections;
    objects.reserve(detections.size());
    for(auto detection : detections){
      // Decode
      Object2D obj;
      obj.box = detection.bbox;
      obj.label = BYTETracker::class_to_int[detection.results[0].id];
      obj.prob = detection.results[0].score;
      obj.time_ms = detection.header.stamp.sec*1000 + detection.header.stamp.nanosec/1e+6;

      objects.push_back(obj);
    }
    #ifdef DEBUG
      std::cout << "Number of objects: " << objects.size() << std::endl;
    #endif
}

// %%%%%%%%%% Callbacks

void BBTracker::add_detection3D(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message)
{
  #ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detections_message->header.frame_id.c_str());
  #endif
  _num_detections++;
  if (_num_detections % 50 == 0)
  {
      RCLCPP_INFO(this->get_logger(), "Processing Detection number %d, Update number %d (Ideally %lu fps)", _num_detections, _num_updates, _num_updates * 1000000 / _total_ms);
  }
  if (detections_message->detections.empty())
    return;

  // Decode detections
  auto start = chrono::system_clock::now();

  // Move all the detections to a common fixed frame
  if(detections_message->header.frame_id != _fixed_frame){
    change_frame(detections_message, _fixed_frame);
  }

  vector<Object3D> objects3D;
  // Put detection3d array inside the object structure
  decode_detections(detections_message, objects3D);

  auto end = chrono::system_clock::now();
  _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();

  update_tracker(objects3D);
}

void BBTracker::add_detection2D(std::shared_ptr<vision_msgs::msg::Detection2DArray> detections_message)
{
  #ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "I heard from: '%s'", detections_message->header.frame_id.c_str());
  #endif
  _num_detections++;
  if (_num_detections % 50 == 0)
  {
      RCLCPP_INFO(this->get_logger(), "Processing Detection number %d, Update number %d (Ideally %lu fps)", _num_detections, _num_updates, _num_updates * 1000000 / _total_ms);
  }
  if (detections_message->detections.empty())
    return;

  // Decode detections
  auto start = chrono::system_clock::now();

  vector<Object2D> objects2D;
  // Put detection2d array inside the object structure
  decode_detections(detections_message, objects2D);

  auto end = chrono::system_clock::now();
  _total_ms = _total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();

  update_tracker(objects2D);
}

void BBTracker::add_detection2D_image(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info, const sensor_msgs::msg::Image::ConstSharedPtr& image) {
  cv_bridge::CvImagePtr cv_ptr;
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info);
  auto projMat = cam_model.projectionMatrix();
  auto trackedObj = this->_tracker.getTrackedObj();

  geometry_msgs::msg::TransformStamped tf_result;
  try {
    tf_result = _tf_buffer.lookupTransform(camera_info->header.frame_id, _fixed_frame, rclcpp::Time(0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_INFO_STREAM(this->get_logger(), "No transform exists for the given tfs: " << _fixed_frame << " - " << camera_info->header.frame_id);
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

  tf2::Matrix3x3 rot_m(q);
  
  VIEW_MATRIX vMat;
  vMat << rot_m[0][0], rot_m[0][1], rot_m[0][2], p.x(),
          rot_m[1][0], rot_m[1][1], rot_m[1][2], p.y(),
          rot_m[2][0], rot_m[2][1], rot_m[2][2], p.z(),
          0.0,          0.0,          0.0,        1.0;


  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
    for(auto det : detection_msg->detections)
      cv::ellipse(cv_ptr->image, Point(det.bbox.center.x, det.bbox.center.y),
              Size(det.bbox.size_x/2.0, det.bbox.size_y/2.0), 0, 0,
              360, CV_RGB(255, 0, 0),
              1, LINE_AA);
    for(auto obj :trackedObj)
      draw_ellipse(cv_ptr, obj, projMat, vMat);

    cv::imshow("Image Window", cv_ptr->image);
    cv::waitKey(1);

  }
  catch (cv_bridge::Exception& e)
  {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
  return;
}

void BBTracker::draw_ellipse(cv_bridge::CvImagePtr image_ptr, STrack obj, cv::Matx34d projMat, VIEW_MATRIX vMat){
  // To create a matrix representing the quadric of the ellipsoid use 
  // [1/a 0 0 cx]
  // [0 1/b 0 cy]
  // [0 0 1/c cz]
  // [0  0  0  1]
  // With a b c semiaxes

  // TODO: try with method described in "Factorization based structure from motion with object priors" by Paul Gay et al.

  // Create an Eigen 3x4 matrix and copy the values from cv::Matx34d
  Eigen::Matrix<float, 3, 4, Eigen::RowMajor> P;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
          P(i, j) = float(projMat(i, j));
      }
  }

  // 1 >>> Transform bbox into ellipsoid 3D
  float a,b,c,cx,cy,cz;

  vector<float> minwdh = obj.minwdh;

  a = minwdh[3]/2;
  b = minwdh[4]/2;
  c = minwdh[5]/2;

  cx = minwdh[0] + a;
  cy = minwdh[1] + b;
  cz = minwdh[2] + c;

  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> quadric;
  quadric << 1.0f/a, 0.0f, 0.0f, cx,
            0.0f, 1.0f/b, 0.0f, cy,
            0.0f, 0.0f, 1.0f/c, cz,
            0.0f, 0.0f, 0.0f, 1.0f;


  // 2 >>> Transform Ellipsoid 3D in ellipse 2D
  float ea, eb, ecx, ecy, theta; // Variables of the ellipse
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> conic;
  // Eigen::Matrix<float, 3, 3, Eigen::RowMajor> adj_conic;
  conic = P * vMat * quadric * vMat.transpose() * P.transpose();
  // conic = adj_conic * 1.0/adj_conic.determinant();


  // 3 >>> Compute parameters from conic form of ellipse
  float A,B,C,D,E,F;
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> M0;
  Eigen::Matrix<float, 2, 2, Eigen::RowMajor> M;

  A = conic(0,0);
  B = conic(0,1)+conic(1,0);
  C = conic(1,1);
  D = conic(0,2)+conic(2,0);
  E = conic(1,2)+conic(2,1);
  F = conic(2,2);

  M0 << F, D/2.0f, E/2.0f,
        D/2.0f, A, B/2.0f,
        E/2.0f, B/2.0f, C;

  M <<  A, B/2.0f,
        B/2.0f, C;

  Eigen::EigenSolver<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>> solver(M);
  auto eigenvalues = solver.eigenvalues().real();

  ea = sqrt(-M0.determinant()/(M.determinant()*eigenvalues(0)));
  eb = sqrt(-M0.determinant()/(M.determinant()*eigenvalues(1)));
  ecx = (B*E - 2.0f*C*D) / (4.0f*A*C - B*B);
  ecy = (B*D - 2.0f*A*E) / (4.0f*A*C - B*B);
  theta = atan(B / (A - C)) / 2.0f;


  std::cout << "quadric: \n" << quadric << "\n conic: \n" << conic << "\n\tCenter: (" << ecx << " , " << ecy << ")" << std::endl;

  // 4 >>> Draw the ellipse
  if (ecx-ea > 0 && ecx+ea < image_ptr->image.rows && ecy-eb > 0 && ecy+eb < image_ptr->image.cols){
    cv::ellipse(image_ptr->image, Point(ecx, ecy),
                Size(ea, eb), theta, 0,
                360, CV_RGB(0, 0, 255),
                1, LINE_AA);
  }
}
// %%%%%%%%%% Visualization

void BBTracker::publish_stracks(vector<STrack*>& output_stracks){
  vision_msgs::msg::Detection3DArray out_message;
  geometry_msgs::msg::PoseArray poses_message;
  visualization_msgs::msg::MarkerArray path_markers;
  visualization_msgs::msg::MarkerArray text_markers;


  out_message.header.stamp = get_clock()->now();
  out_message.header.frame_id = _fixed_frame;
  poses_message.header = out_message.header;

  path_markers.markers.clear();
  text_markers.markers.clear();
  path_markers.markers.reserve(output_stracks.size());
  text_markers.markers.reserve(output_stracks.size());
  out_message.detections.reserve(output_stracks.size());
  poses_message.poses.reserve(output_stracks.size());
  for (unsigned int i = 0; i < output_stracks.size(); i++)
  {
    auto current_track = output_stracks[i];
    vector<float> minwdh = current_track->minwdh;

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
    hypothesis.id = current_track->class_name;
    hypothesis.score = current_track->score;
    single_det.results.push_back(hypothesis);

    out_message.detections.push_back(single_det);
    // Publish a path for each track, with relative text
    visualization_msgs::msg::Marker text;
    visualization_msgs::msg::Marker path_marker = createPathMarker(current_track, out_message.header, single_det.bbox.center.position, text);
    text.pose.position.z += single_det.bbox.size.z/2;
    path_markers.markers.push_back(path_marker);
    text_markers.markers.push_back(text);
    poses_message.poses.push_back(single_det.bbox.center);
  }

  _det_publisher->publish(out_message);
  _det_poses_publisher->publish(poses_message);
  _path_publisher->publish(path_markers);
  _text_publisher->publish(text_markers);

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
    track->text_marker.text = track->class_name;
    track->text_marker.text.append("-").append(::to_string(track->track_id));
  }
  track->path_marker.points.push_back(last_point);
  // path_marker.colors.push_back(s_color);
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
  return 0;
}