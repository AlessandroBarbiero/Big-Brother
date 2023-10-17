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

  // _sync_det_camera = std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image>>(
  // _detection2d, _camera_info, _camera_image, 100);

  // _sync_det_camera->registerCallback(std::bind(&BBTracker::add_detection2D_image, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));       

  _test_projection = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image>>(_camera_info, _camera_image, 100);
  _test_projection->registerCallback(std::bind(&BBTracker::test_ellipse_project, this, std::placeholders::_1, std::placeholders::_2));

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

  // TODO: decomment this
  // publish_stracks(output_stracks);

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

void BBTracker::change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, const std::string& new_frame){
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

// TODO: delete
// Function that prints the posizion of the mouse on click over image
void mouse_callback(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == EVENT_LBUTTONDOWN )
  {
    cout << "----> Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  }
}

void BBTracker::add_detection2D_image(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info, const sensor_msgs::msg::Image::ConstSharedPtr& image) {
  cv_bridge::CvImagePtr cv_ptr;
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info);
  auto projMat = cam_model.projectionMatrix();

  std::vector<STrack> real_trackedObj = this->_tracker.getTrackedObj();
  // TODO: delete Fake points to test the draw
  std::vector<STrack> trackedObj;
  trackedObj.push_back(real_trackedObj[0]);
  trackedObj[0].minwdh = {-30,15,0,2,2,2};


  std::vector<STrack*> trackedObj_draw;
  for(size_t i = 0 ; i< trackedObj.size(); i++)
    trackedObj_draw.push_back(&trackedObj[i]);
  publish_stracks(trackedObj_draw);

  // auto det_mess = std::make_shared<vision_msgs::msg::Detection3DArray>();
  // convert_into_detections(trackedObj_draw, det_mess.get());
  // change_frame(det_mess, camera_info->header.frame_id);
  // det_mess->header.stamp = camera_info->header.stamp;
  // _det_publisher->publish(*det_mess.get());
  // ---------

  TRANSFORMATION vMat = getViewMatrix(_fixed_frame, camera_info->header.frame_id);

  //std::cout << "View matrix, inverse of camera matrix: \n" << vMat << std::endl;

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

  // K_MATRIX K;
  // int k_index = 0;
  // for (int i = 0; i < 3; ++i) {
  //     for (int j = 0; j < 3; ++j) {
  //         K(i, j) = camera_info->k[k_index];
  //         k_index++;
  //     }
  // }

  // Try to compute a Projection matrix from the fixed frame
  // PROJ_MATRIX RT, P;
  // RT << rot_m[0][0], rot_m[0][1], rot_m[0][2], p.x(),
  //       rot_m[1][0], rot_m[1][1], rot_m[1][2], p.y(),
  //       rot_m[2][0], rot_m[2][1], rot_m[2][2], p.z();
  // P = K * RT;

  // std::cout << "K matrix: \n" << K << "\nP matrix: \n" << P << std::endl;

  // Try to use a 4x4 Projection matrix 
  // TRANSFORMATION proj_matrix;
  // proj_matrix.setZero();
  // auto cx = static_cast<float>(camera_info->p[2]);
  // auto cy = static_cast<float>(camera_info->p[6]);
  // auto fx = static_cast<float>(camera_info->p[0]);
  // auto fy = static_cast<float>(camera_info->p[5]);
  // float far_plane = 100.0f;
  // float near_plane = 0.01f;
  // proj_matrix(0,0) = 2.0f * fx / camera_info->width;
  // proj_matrix(1,1) = 2.0f * fy / camera_info->height;
  // proj_matrix(0,2) = 2.0f * (0.5f - cx / camera_info->width);
  // proj_matrix(1,2) = 2.0f * (cy / camera_info->height - 0.5f);
  // proj_matrix(2,2) = -(far_plane + near_plane) / (far_plane - near_plane);
  // proj_matrix(2,3) = -2.0f * far_plane * near_plane / (far_plane - near_plane);
  // proj_matrix(3,2) = -1;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, image->encoding);

    // Draw ellipses for incoming detections
    // for(auto det : detection_msg->detections)
    //   cv::ellipse(cv_ptr->image, Point(det.bbox.center.x, det.bbox.center.y),
    //           Size(det.bbox.size_x/2.0, det.bbox.size_y/2.0), 0, 0,
    //           360, CV_RGB(255, 0, 0),
    //           1, LINE_AA);
    // Draw ellipses for tracked objects
    for(auto obj :trackedObj)
      draw_ellipse(cv_ptr, obj, P, vMat);
    // for(auto det : det_mess->detections){
    //   draw_ellipse(cv_ptr, det, P, proj_matrix, transform, cam_model);
    // }
    std::string window_name = "Image Window";
    cv::imshow(window_name, cv_ptr->image);
    cv::setMouseCallback(window_name, mouse_callback);
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
  // TODO: delete Fake points to test the draw
  // std::vector<STrack> trackedObj;
  // trackedObj.push_back(real_trackedObj[0]);
  // trackedObj[0].minwdh = {-30,15,0,2,2,2};

  // std::vector<STrack*> trackedObj_draw;
  // for(size_t i = 0 ; i< trackedObj.size(); i++)
  //   trackedObj_draw.push_back(&trackedObj[i]);
  // publish_stracks(trackedObj_draw);
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
  // cout << "projection mat:\n" << P <<
  //       "view mat:\n"<< vMat << endl;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
    // Draw ellipses for tracked objects
    for(auto obj :trackedObj)
      draw_ellipse(cv_ptr, obj, P, vMat);

    std::string window_name = "Image Window";
    cv::imshow(window_name, cv_ptr->image);
    cv::setMouseCallback(window_name, mouse_callback);
    cv::waitKey(1);

  }
  catch (cv_bridge::Exception& e)
  {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
  return;
}

// Get the view matrix that transform a tf to another 
// vMat = [Rot  | center_of_other_tf_in_new_tf]
//        [0    |               1             ]
TRANSFORMATION BBTracker::getViewMatrix(std::string from_tf, std::string camera_tf){
  TRANSFORMATION vMat;

  geometry_msgs::msg::TransformStamped tf_result;
  try {
    tf_result = _tf_buffer.lookupTransform(camera_tf, from_tf, rclcpp::Time(0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_INFO_STREAM(this->get_logger(), "No transform exists for the given tfs: " << from_tf << " - " << camera_tf);
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

/**
 * Compute center, axes lengths, and orientation of an ellipse in dual form.
 * Method used in 3D Object Localisation from Multi-view Image Detections (TPAMI 2017)
 *
 * @param C The ellipse in dual form [3x3].
 *
 * @return A tuple containing:
 *   - centre: Ellipse center in Cartesian coordinates [2x1].
 *   - axes: Ellipse axes lengths [2x1].
 *   - R_matrix: Ellipse orientation as a rotation matrix [2x2].
 */
std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix2f> dualEllipseToParameters(Eigen::Matrix3f C) {
  if (C(2, 2) != 0) {
      C /= -C(2, 2);
  }

  // Take the first two elements of last column and put in column vector
  Eigen::Vector2f centre = -C.block<2, 1>(0, 2);

  // T = [  I | -centre ]
  //     [  0 |     1   ]
  Eigen::Matrix3f T;
  T << Eigen::Matrix2f::Identity(), -centre, 0, 0, 1;

  Eigen::Matrix3f C_centre = T * C * T.transpose();
  C_centre = 0.5 * (C_centre + C_centre.transpose());

  cout << "C_sub = \n" << C_centre.block<2, 2>(0, 0) << endl;

  Eigen::EigenSolver<Eigen::Matrix2f> eigensolver(C_centre.block<2, 2>(0, 0));
  Eigen::Vector2f D = eigensolver.eigenvalues().real().cwiseSqrt();
  Eigen::Matrix2f V = eigensolver.eigenvectors().real();

  // cout << "Ellipse parameters:" << 
  //       "\n\tcentre:\n" << centre << 
  //       "\n\tT:\n" << T <<
  //       "\n\tD:\n" << D <<
  //       "\n\tV:\n" << V << endl;

  return std::make_tuple(centre, D, V);
}

/**
 * Compose a dual ellipsoid matrix given axes, rotation matrix, and center.
 *
 * @param axes   A vector of axis lengths.
 * @param R      The rotation matrix.
 * @param center The center vector.
 * @return       The composed dual ellipsoid matrix.
 */
Eigen::Matrix4f composeDualEllipsoid(const std::vector<float>& axes, const Eigen::Matrix3f& R, const Eigen::Vector3f& center) {
  Eigen::Matrix4f Q;
  Q.setIdentity();
  for (int i = 0; i < 3; ++i) {
    Q(i, i) = axes[i] * axes[i];
  }
  Q(3, 3) = -1.0;

  Eigen::Matrix4f Re_w = Eigen::Matrix4f::Identity();
  Re_w.block<3, 3>(0, 0) = R.transpose();

  Eigen::Matrix4f T_center_inv = Eigen::Matrix4f::Identity();
  T_center_inv.block<3, 1>(0, 3) = center;

  Eigen::Matrix4f Q_star = T_center_inv * Re_w.transpose() * Q * Re_w * T_center_inv.transpose();
  return Q_star;
}

Eigen::Matrix<float,3,3,Eigen::RowMajor> getRotationMatrix(float yaw_angle){
  Eigen::Matrix<float,3,3,Eigen::RowMajor> R;
  // Quaternion for general form
  // tf2::Quaternion quat;
  // quat.setRPY(0,0,yaw_angle);
  // tf2::Matrix3x3 rot_ellipsoid(quat);
  // R <<  rot_ellipsoid[0][0], rot_ellipsoid[0][1], rot_ellipsoid[0][2],
  //       rot_ellipsoid[1][0], rot_ellipsoid[1][1], rot_ellipsoid[1][2],
  //       rot_ellipsoid[2][0], rot_ellipsoid[2][1], rot_ellipsoid[2][2];

  R <<  cos(yaw_angle), -sin(yaw_angle), 0,
        sin(yaw_angle), cos(yaw_angle), 0,
        0, 0, 1;
  return R;
}

// Transform the state into the ellipse state inside the image represented by the View Matrix and the Projection matrix
// It uses different matrix computations, clearer than v2 but slower
// Returns ellipse variables:
//  - x coordinate center
//  - y coordinate center
//  - semi-axis a
//  - semi-axis b
Eigen::Vector4f ellipseFromEllipsoidv1(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION vMat, PROJ_MATRIX P){
  // State
  float x = state(0),
  y =       state(1),
  theta =   state(2),
  l_ratio = state(3),
  d_ratio = state(4),
  h =       state(5);
  // v =       state(6),
  // w =       state(7);

  // 1 >>> Transform bbox into ellipsoid 3D

  float z = h/2,
  a = l_ratio*h/2, 
  b = d_ratio*h/2, 
  c = h/2;
  std::vector<float> semi_axis = {a,b,c};
  Eigen::Matrix<float,3,3,Eigen::RowMajor> R = getRotationMatrix(theta);
  Eigen::Vector3f center_vec;
  center_vec << x,y,z;
  // This is the dual ellipsoid in fixed_frame
  Eigen::Matrix4f dual_ellipsoid_fix_f = composeDualEllipsoid(semi_axis, R, center_vec);
  Eigen::Matrix4f dual_ellipsoid;
  // The object is in the _fixed_frame tf, I want to convert its coordinate into camera frame before multiply for projection matrix
  dual_ellipsoid = vMat * dual_ellipsoid_fix_f * vMat.transpose();

  // 2 >>> Transform Ellipsoid 3D in ellipse 2D

  Eigen::Matrix3f dual_ellipse;
  dual_ellipse = P * dual_ellipsoid * P.transpose();

  // 3 >>> Compute parameters from conic form of ellipse

  std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix2f> ellipse_params = dualEllipseToParameters(dual_ellipse);
  Eigen::Vector2f e_center = std::get<0>(ellipse_params);
  Eigen::Vector2f e_axis = std::get<1>(ellipse_params);
  Eigen::Matrix2f e_rotmat = std::get<2>(ellipse_params);
  // cout<<"ellipse center:\n"<<e_center<<
  //       "\nellipse axis:\n" <<e_axis <<
  //       "\nellipse rotation matrix:\n" << e_rotmat << endl;

  float etheta;
  etheta = atan2(e_rotmat(1, 0), e_rotmat(0, 0));  // Not used yet
  // Ensure theta is in the range [-pi, pi]
  if (etheta < -M_PI) {
      etheta += 2 * M_PI;
  } else if (etheta > M_PI) {
      etheta -= 2 * M_PI;
  }

  Eigen::Vector4f result;
  result << 
      e_center(0),
      e_center(1),
      e_axis(0),
      e_axis(1);
  return result;
}

// Transform the state into the ellipse state inside the image represented by the View Matrix and the Projection matrix
// It uses a single shot equation to do so.
// Returns ellipse variables:
//  - x coordinate center
//  - y coordinate center
//  - semi-axis a
//  - semi-axis b
Eigen::Vector4f ellipseFromEllipsoidv2(Eigen::Matrix<float, 1, 8> state, TRANSFORMATION vMat, PROJ_MATRIX P){
  cout << "State: \n" << state << endl;
  Eigen::Vector4f result;
  // State
  float x = state(0),
  y =       state(1),
  theta =   state(2),
  l_ratio = state(3),
  d_ratio = state(4),
  h =       state(5),
  v =       state(6),
  w =       state(7),
  // View
  vr00 = vMat(0,0),
  vr01 = vMat(0,1),
  vr02 = vMat(0,2),
  vr10 = vMat(1,0),
  vr11 = vMat(1,1),
  vr12 = vMat(1,2),
  vr20 = vMat(2,0),
  vr21 = vMat(2,1),
  vr22 = vMat(2,2),
  vtx = vMat(0,3),
  vty = vMat(1,3),
  vtz = vMat(2,3),
  // Projection
  fx = P(0,0),
  fy = P(1,1),
  cx = P(0,2),
  cy = P(1,2);

  result <<
	 ((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*(-vr20*sin(theta)+vr21*cos(theta))*(-(cx*vr20+fx*vr00)*sin(theta)+(cx*vr21+fx*vr01)*cos(theta))+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*(vr20*cos(theta)+vr21*sin(theta))*((cx*vr20+fx*vr00)*cos(theta)+(cx*vr21+fx*vr01)*sin(theta))+(1.0/4.0)*pow(h,2)*vr22*(cx*vr22+fx*vr02)-((1.0/2.0)*h*vr22+vr20*x+vr21*y+vtz)*(cx*vtz+fx*vtx+(1.0/2.0)*h*(cx*vr22+fx*vr02)+x*(cx*vr20+fx*vr00)+y*(cx*vr21+fx*vr01)))/((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-vr20*sin(theta)+vr21*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow(vr20*cos(theta)+vr21*sin(theta),2)+ (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2)) ,
	 ((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-vr20*sin(theta) + vr21*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*(vr20*cos(theta) + vr21*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*vr22*(cy*vr22 + fy*vr12) - ((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz)*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)))/((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2)) ,
	(1.0/2.0)*M_SQRT2*sqrt(sqrt(sqrt(pow((-1.0/4.0*pow(d_ratio,2)*pow(h,2)*pow(-(cx*vr20+fx*vr00)*sin(theta)+(cx*vr21+fx*vr01)*cos(theta),2)-1.0/4.0*pow(d_ratio,2)*pow(h,2)*pow(-(cy*vr20+fy*vr10)*sin(theta)+(cy*vr21+fy*vr11)*cos(theta),2)-1.0/4.0*pow(h,2)*pow(l_ratio,2)*pow((cx*vr20+fx*vr00)*cos(theta)+(cx*vr21+fx*vr01)*sin(theta),2)-1.0/4.0*pow(h,2)*pow(l_ratio,2)*pow((cy*vr20+fy*vr10)*cos(theta)+(cy*vr21+fy*vr11)*sin(theta),2)-1.0/4.0*pow(h,2)*pow(cx*vr22+fx*vr02,2)-1.0/4.0*pow(h,2)*pow(cy*vr22+fy*vr12,2)
-
sqrt((pow((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-(cx*vr20+fx*vr00)*sin(theta)+(cx*vr21+fx*vr01)*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow((cx*vr20+fx*vr00)*cos(theta)+(cx*vr21+fx*vr01)*sin(theta),2)+(1.0/4.0)*pow(h,2)*pow(cx*vr22+fx*vr02,2)-pow(cx*vtz+fx*vtx+(1.0/2.0)*h*(cx*vr22+fx*vr02)+x*(cx*vr20+fx*vr00)+y*(cx*vr21+fx*vr01),2),2)-2*((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-(cx*vr20+fx*vr00)*sin(theta)+(cx*vr21+fx*vr01)*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow((cx*vr20+fx*vr00)*cos(theta)+(cx*vr21+fx*vr01)*sin(theta),2)+(1.0/4.0)*pow(h,2)*pow(cx*vr22+fx*vr02,2)-pow(cx*vtz+fx*vtx+(1.0/2.0)*h*(cx*vr22+fx*vr02)+x*(cx*vr20+fx*vr00)+y*(cx*vr21+fx*vr01),2))*((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-(cy*vr20+fy*vr10)*sin(theta)+(cy*vr21+fy*vr11)*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow((cy*vr20+fy*vr10)*cos(theta)+(cy*vr21+fy*vr11)*sin(theta),2)+(1.0/4.0)*pow(h,2)*pow(cy*vr22+fy*vr12,2)-pow(cy*vtz+fy*vty+(1.0/2.0)*h*(cy*vr22+fy*vr12)+x*(cy*vr20+fy*vr10)+y*(cy*vr21+fy*vr11),2))+pow((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-(cy*vr20+fy*vr10)*sin(theta)+(cy*vr21+fy*vr11)*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow((cy*vr20+fy*vr10)*cos(theta)+(cy*vr21+fy*vr11)*sin(theta),2)+(1.0/4.0)*pow(h,2)*pow(cy*vr22+fy*vr12,2)-pow(cy*vtz+fy*vty+(1.0/2.0)*h*(cy*vr22+fy*vr12)+x*(cy*vr20+fy*vr10)+y*(cy*vr21+fy*vr11),2),2)+4*pow((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*(-(cx*vr20+fx*vr00)*sin(theta)+(cx*vr21+fx*vr01)*cos(theta))*(-(cy*vr20+fy*vr10)*sin(theta)+(cy*vr21+fy*vr11)*cos(theta))+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*((cx*vr20+fx*vr00)*cos(theta)+(cx*vr21+fx*vr01)*sin(theta))*((cy*vr20+fy*vr10)*cos(theta)+(cy*vr21+fy*vr11)*sin(theta))+(1.0/4.0)*pow(h,2)*(cx*vr22+fx*vr02)*(cy*vr22+fy*vr12)-(cx*vtz+fx*vtx+(1.0/2.0)*h*(cx*vr22+fx*vr02)+x*(cx*vr20+fx*vr00)+y*(cx*vr21+fx*vr01))*(cy*vtz+fy*vty+(1.0/2.0)*h*(cy*vr22+fy*vr12)+x*(cy*vr20+fy*vr10)+y*(cy*vr21+fy*vr11)),2))/pow((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-vr20*sin(theta)+vr21*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow(vr20*cos(theta)+vr21*sin(theta),2)+(1.0/4.0)*pow(h,2)*pow(vr22,2)-pow((1.0/2.0)*h*vr22+vr20*x+vr21*y+vtz,2),2))*((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-vr20*sin(theta)+vr21*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow(vr20*cos(theta)+vr21*sin(theta),2)+(1.0/4.0)*pow(h,2)*pow(vr22,2)-pow((1.0/2.0)*h*vr22+vr20*x+vr21*y+vtz,2))+pow(cx*vtz+fx*vtx+(1.0/2.0)*h*(cx*vr22+fx*vr02)+x*(cx*vr20+fx*vr00)+y*(cx*vr21+fx*vr01),2)+pow(cy*vtz+fy*vty+(1.0/2.0)*h*(cy*vr22+fy*vr12)+x*(cy*vr20+fy*vr10)+y*(cy*vr21+fy*vr11),2))/((1.0/4.0)*pow(d_ratio,2)*pow(h,2)*pow(-vr20*sin(theta)+vr21*cos(theta),2)+(1.0/4.0)*pow(h,2)*pow(l_ratio,2)*pow(vr20*cos(theta)+vr21*sin(theta),2)+(1.0/4.0)*pow(h,2)*pow(vr22,2)-pow((1.0/2.0)*h*vr22+vr20*x+vr21*y+vtz,2)),2)))),
  	 (1.0/2.0)*M_SQRT2*sqrt((-1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) - 1.0/4.0*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) - 1.0/4.0*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - 1.0/4.0*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) + sqrt((pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2), 2) - 2*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cx*vr22 + fx*vr02, 2) - pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2)) + pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(cy*vr22 + fy*vr12, 2) - pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2), 2) + 4*pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*(-(cx*vr20 + fx*vr00)*sin(theta) + (cx*vr21 + fx*vr01)*cos(theta))*(-(cy*vr20 + fy*vr10)*sin(theta) + (cy*vr21 + fy*vr11)*cos(theta)) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*((cx*vr20 + fx*vr00)*cos(theta) + (cx*vr21 + fx*vr01)*sin(theta))*((cy*vr20 + fy*vr10)*cos(theta) + (cy*vr21 + fy*vr11)*sin(theta)) + (1.0/4.0)*pow(h, 2)*(cx*vr22 + fx*vr02)*(cy*vr22 + fy*vr12) - (cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01))*(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11)), 2))/pow((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2), 2))*((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2)) + pow(cx*vtz + fx*vtx + (1.0/2.0)*h*(cx*vr22 + fx*vr02) + x*(cx*vr20 + fx*vr00) + y*(cx*vr21 + fx*vr01), 2) + pow(cy*vtz + fy*vty + (1.0/2.0)*h*(cy*vr22 + fy*vr12) + x*(cy*vr20 + fy*vr10) + y*(cy*vr21 + fy*vr11), 2))/((1.0/4.0)*pow(d_ratio, 2)*pow(h, 2)*pow(-vr20*sin(theta) + vr21*cos(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(l_ratio, 2)*pow(vr20*cos(theta) + vr21*sin(theta), 2) + (1.0/4.0)*pow(h, 2)*pow(vr22, 2) - pow((1.0/2.0)*h*vr22 + vr20*x + vr21*y + vtz, 2))) ;
  return result;
}

void BBTracker::draw_ellipse(cv_bridge::CvImagePtr image_ptr, STrack obj, PROJ_MATRIX P, TRANSFORMATION vMat){
  // Took inspiration from "Factorization based structure from motion with object priors" by Paul Gay et al.
  float w,d,h,cx,cy,cz;

  vector<float> minwdh = obj.minwdh;

  w = minwdh[3];
  d = minwdh[4];
  h = minwdh[5];

  cx = minwdh[0] + w/2;
  cy = minwdh[1] + d/2;
  cz = minwdh[2] + h/2;

  std::cout << 
      "Center in fixed frame: (" << cx << " , " << cy << " , " << cz << ")" <<
      "\nSizes in fixed frame: (" << w << " , " << d << " , " << h << ")" << std::endl;

  // Filter out objects behind the camera
  float check = vMat(2,0)*cx +vMat(2,1)*cy + vMat(2,2)*cz + vMat(2,3);
  if(check < 0)
    return;

  Eigen::Matrix<float,1,8> ellipsoid_state;
  ellipsoid_state << cx,cy,obj.theta,w/h,d/h,h,0,0;
  Eigen::Vector4f ellipse_state = ellipseFromEllipsoidv1(ellipsoid_state, vMat, P);

  float ea, eb, ecx, ecy, theta; // Variables of the ellipse

  ecx = ellipse_state(0);
  ecy = ellipse_state(1);
  ea = ellipse_state(2);
  eb = ellipse_state(3);
  // ea = 50;
  // eb = 50;
  theta = 0;

    std::cout << 
  //   "dual ellipsoid: \n" << dual_ellipsoid << 
  //   "\ndual ellipse: \n" << dual_ellipse << 
    "\n\tCenter: (" << ecx << " , " << ecy << ")" << 
    "\n\tSemiAxes: (" << ea << " , " << eb << ")" << std::endl;

  if(ea < 0 || eb < 0){
    return;
  }

  // 4 >>> Draw the ellipse
  if (ecx-ea > 0 && ecx+ea < image_ptr->image.cols && ecy-eb > 0 && ecy+eb < image_ptr->image.rows){
    // std::cout << "Ellipse in the image" << std::endl;
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

  convert_into_detections(output_stracks, &out_message);

  poses_message.header = out_message.header;

  path_markers.markers.clear();
  text_markers.markers.clear();
  path_markers.markers.reserve(output_stracks.size());
  text_markers.markers.reserve(output_stracks.size());
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

    auto single_det = out_message.detections[i];

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

void BBTracker::convert_into_detections(vector<STrack*>& in_stracks, vision_msgs::msg::Detection3DArray* out_message){
  out_message->header.stamp = get_clock()->now();
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
    hypothesis.id = current_track->class_name;
    hypothesis.score = current_track->score;
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
  getchar();
  return 0;
}