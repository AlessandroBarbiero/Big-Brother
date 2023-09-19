#include <bb_utils/bb_benchmark.hpp>

BBBenchmark::BBBenchmark()
: Node("bb_benchmark"), _tf_buffer(this->get_clock()), _tf_listener(_tf_buffer)
{
  // ROS Parameters
  auto fps_desc = rcl_interfaces::msg::ParameterDescriptor{};
  fps_desc.description = "Frequency of the Tracker, number of executions per second";
  auto m_thresh_desc = rcl_interfaces::msg::ParameterDescriptor{};
  m_thresh_desc.description = "This threshold is used to establish the perfect match between existing tracks and ground truth.";
  auto fixed_frame_desc = rcl_interfaces::msg::ParameterDescriptor{};
  fixed_frame_desc.description = "The fixed frame the bb_benchmark has to use, all the tracks has to give a transform for this frame, usually the frame of the ground truth";
  auto s_range_desc = rcl_interfaces::msg::ParameterDescriptor{};
  s_range_desc.description = "Set to true to show the range of sensors, this is the range within which the accuracy is computed";
  auto alpha_range_desc = rcl_interfaces::msg::ParameterDescriptor{};
  alpha_range_desc.description = "Alpha value for sensor range markers, 1 means thay are solid, 0 they are invisible";

  auto camera_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  camera_list_desc.description = "List of the camera info topics";
  auto cameras_max_dist_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cameras_max_dist_desc.description = "List of the max distances for object detected by cameras, respect the order of 'camera_list'";

  auto lidar_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_list_desc.description = "List of the lidar tf";
  auto lidar_max_dist_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_max_dist_desc.description = "List of the max distances for object detected by lidars, respect the order of 'lidar_list'";

  _cameras = {"cam1"};
  _cameras_max_dist = {0};

  _lidars = {"lid1"};
  _lidars_max_dist = {0};

  this->declare_parameter("fps", 30, fps_desc);
  this->declare_parameter("match_thresh", 0.8, m_thresh_desc);
  this->declare_parameter("fixed_frame", "map", fixed_frame_desc);
  this->declare_parameter("show_range", true, s_range_desc);
  this->declare_parameter("alpha_range", 0.1, alpha_range_desc);
  this->declare_parameter("camera_list", _cameras, camera_list_desc);
  this->declare_parameter("camera_max_distances", _cameras_max_dist, cameras_max_dist_desc);
  this->declare_parameter("lidar_list", _lidars, lidar_list_desc);
  this->declare_parameter("lidar_max_distances", _lidars_max_dist, lidar_max_dist_desc);

  _fps =              get_parameter("fps").as_int();
  _match_thresh =     get_parameter("match_thresh").as_double();
  _fixed_frame =      get_parameter("fixed_frame").as_string();
  _show_range  =      get_parameter("show_range").as_bool();
  _alpha_range =      get_parameter("alpha_range").as_double();
  _cameras =           get_parameter("camera_list").as_string_array();
  _cameras_max_dist =  get_parameter("camera_max_distances").as_integer_array();
  _lidars =            get_parameter("lidar_list").as_string_array();
  _lidars_max_dist =   get_parameter("lidar_max_distances").as_integer_array();

  rclcpp::QoS qos_transient_local(rclcpp::KeepLast(10));
  qos_transient_local.transient_local();


  // Add a subscriber for each camera info topic 
  int id = 0;
  for (auto camera_topic : _cameras){
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub;
    std::function<void(std::shared_ptr<sensor_msgs::msg::CameraInfo>)> callback_fun = std::bind(
      &BBBenchmark::camera_info_callback, this, _1, id, _cameras_max_dist[id]);
    
    cam_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_topic, rclcpp::QoS(rclcpp::KeepLast(1)), callback_fun);
    _cameras_sub.push_back(cam_sub);
    _last_transform_camera.push_back(geometry_msgs::msg::Transform());
    _tf2_transform_camera.push_back(tf2::Transform());
    _camera_models.push_back(image_geometry::PinholeCameraModel());
    id++;
  }

  _lidar_ready = false;

  if(_show_range){
    _sensor_range_pub = this->create_publisher<visualization_msgs::msg::Marker>("benchmark/sensor_range", qos_transient_local);
    
    publish_lidar_range(_lidars, _lidars_max_dist);
  }

  _tracker_out_sub = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "bytetrack/active_tracks", 10, std::bind(&BBBenchmark::tracker_out, this, _1));

  _static_ground_truth_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "carla/markers/static", qos_transient_local, std::bind(&BBBenchmark::save_static_gt, this, _1));
  _ground_truth_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "carla/markers", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&BBBenchmark::save_gt, this, _1));

  _debug_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("benchmark/debug", 10);


  RCLCPP_INFO(this->get_logger(), "Benchmark Ready!");

}

void BBBenchmark::init_lidar_tf(){
  _last_transform_lidar.resize(_lidars.size());
  _tf2_transform_lidar.resize(_lidars.size());
  geometry_msgs::msg::TransformStamped tf_result;
  for(long unsigned int id=0; id<_lidars.size(); id++){
    try {
      tf_result = _tf_buffer.lookupTransform(_lidars[id], _fixed_frame, rclcpp::Time(0));
    } catch (tf2::TransformException& ex) {
      RCLCPP_INFO_STREAM(this->get_logger(), "No transform exists for the given tfs: " << _fixed_frame << " - " << _lidars[id]);
      return;
    }
    _last_transform_lidar[id] = tf_result.transform;
    tf2::Quaternion q(
      _last_transform_lidar[id].rotation.x,
      _last_transform_lidar[id].rotation.y,
      _last_transform_lidar[id].rotation.z,
      _last_transform_lidar[id].rotation.w
    );
    tf2::Vector3 p(
      _last_transform_lidar[id].translation.x,
      _last_transform_lidar[id].translation.y,
      _last_transform_lidar[id].translation.z
    );
    tf2::Transform transform(q, p);
    _tf2_transform_lidar[id] = transform;
  }
  _lidar_ready=true;
}

void BBBenchmark::save_static_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg){
  if(msg->markers[0].header.frame_id!=_fixed_frame)
    RCLCPP_WARN(this->get_logger(), "static gt should have the same tf as fixed frame");

  _static_objects.reserve(msg->markers.size());
  for(auto marker: msg->markers){
    vision_msgs::msg::BoundingBox3D bbox;
    bbox.center=marker.pose;
    bbox.size=marker.scale;
    _static_objects.push_back(bbox);
  }

  RCLCPP_INFO(this->get_logger(), "Static Objects saved!");
}

void BBBenchmark::save_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg)
{
  _moving_objects = msg;
}

void BBBenchmark::save_gt_bbox(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg)
{
  _moving_objects_bbox.clear();
  _moving_objects_bbox.reserve(msg->markers.size());
  for(auto marker: msg->markers){
    vision_msgs::msg::BoundingBox3D bbox;
    bbox.center=marker.pose;
    bbox.size=marker.scale;
    _moving_objects_bbox.push_back(bbox);
  }

  // RCLCPP_INFO_STREAM(this->get_logger(), "Moving Objects saved! " << _moving_objects_bbox.size() << " Objects");
}

void BBBenchmark::publish_lidar_range(std::vector<std::string> lidars, std::vector<int64_t> lidars_max_dist)
{
    for (long unsigned int i = 0; i < lidars.size(); i++)
    {
        visualization_msgs::msg::Marker marker = getLidarRangeMarker(lidars[i], i, static_cast<float>(lidars_max_dist[i]));
        _sensor_range_pub->publish(marker);
    }
}

visualization_msgs::msg::Marker BBBenchmark::getLidarRangeMarker(std::string frame_id, int id, float range){
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "lidar_range";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0.0);
  marker.color.r = 255;
  marker.color.g = 255;
  marker.color.b = 0;
  marker.color.a = _alpha_range;
  marker.pose = geometry_msgs::msg::Pose();
  marker.scale.x = range*2;
  marker.scale.y = range*2;
  marker.scale.z = range*2;

  return marker;
}

void BBBenchmark::change_frame(std::shared_ptr<vision_msgs::msg::Detection3DArray> old_message, std::string& new_frame){
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

vector<vector<float> > BBBenchmark::ious(vector<vector<float> > &aminmaxs, vector<vector<float> > &bminmaxs)
{
	vector<vector<float> > ious;
	if (aminmaxs.size()*bminmaxs.size() == 0)
		return ious;

	ious.resize(aminmaxs.size());
	for (unsigned int i = 0; i < ious.size(); i++)
	{
		ious[i].resize(bminmaxs.size());
	}

	//bbox_ious
	for (unsigned int k = 0; k < bminmaxs.size(); k++)
	{
		vector<float> ious_tmp;
		float box_volume = (bminmaxs[k][3] - bminmaxs[k][0] + 1)*(bminmaxs[k][4] - bminmaxs[k][1] + 1)*(bminmaxs[k][5] - bminmaxs[k][2] + 1);
		for (unsigned int n = 0; n < aminmaxs.size(); n++)
		{
			float iw = min(aminmaxs[n][3], bminmaxs[k][3]) - max(aminmaxs[n][0], bminmaxs[k][0]) + 1;
			if (iw > 0)
			{
				float id = min(aminmaxs[n][4], bminmaxs[k][4]) - max(aminmaxs[n][1], bminmaxs[k][1]) + 1;
				if(id>0)
				{
					float ih = min(aminmaxs[n][5], bminmaxs[k][5]) - max(aminmaxs[n][2], bminmaxs[k][2]) + 1;
					if(ih > 0)
					{
						//Intersection
						float inter = iw * id * ih;
						//Union
						float ua = (aminmaxs[n][3] - aminmaxs[n][0] + 1)*(aminmaxs[n][4] - aminmaxs[n][1] + 1)*(aminmaxs[n][5] - aminmaxs[n][2] + 1) + box_volume - inter;
						ious[n][k] = inter / ua;
					}
					else
					{
						ious[n][k] = 0.0;
					}
				}
				else
				{
					ious[n][k] = 0.0;
				}
			}
			else
			{
				ious[n][k] = 0.0;
			}
		}
	}

	return ious;
}

void BBBenchmark::tracker_out(std::shared_ptr<vision_msgs::msg::Detection3DArray> detections_message)
{
  if (detections_message->detections.empty())
    //Something
    return;

  // Move all the detections to a common fixed frame
  if(detections_message->header.frame_id != _fixed_frame){
    change_frame(detections_message, _fixed_frame);
  }

  // Convert markers of moving objects in bbox only when it is necessary to compare
  save_gt_bbox(_moving_objects);
  // Put moving and static objects together
  vector<vision_msgs::msg::BoundingBox3D> all_objects(_static_objects);
  all_objects.insert(all_objects.end(), _moving_objects_bbox.begin(), _moving_objects_bbox.end());
  // Retrive objects on cameras and lidars
  vector<vision_msgs::msg::BoundingBox3D> on_camera = filter_camera(all_objects);
  vector<vision_msgs::msg::BoundingBox3D> on_lidar = filter_lidar(all_objects);
  vector<vision_msgs::msg::BoundingBox3D> objects_to_compare = obj_union(on_camera, on_lidar);
  RCLCPP_INFO_STREAM(this->get_logger(), "Objects seen by cameras: " << on_camera.size() << " Objects seen by lidars: " << on_lidar.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "Objects total: " << objects_to_compare.size());
  show_objects(on_camera, "Objects_on_camera");
  show_objects(on_lidar, "Objects_on_lidar");
  show_objects(objects_to_compare, "Objects_in_sensor_range");
  
  //TODO: Compare with ious bounding boxes to see right and wrong detections

}

// Custom comparison function for BoundingBoxes
bool customCompare(const vision_msgs::msg::BoundingBox3D& obj1, const vision_msgs::msg::BoundingBox3D& obj2) {
    if (obj1.center.position.x != obj2.center.position.x) return obj1.center.position.x < obj2.center.position.x;
    if (obj1.center.position.y != obj2.center.position.y) return obj1.center.position.y < obj2.center.position.y;
    return obj1.center.position.z < obj2.center.position.z;

}

vector<vision_msgs::msg::BoundingBox3D> BBBenchmark::obj_union(vector<vision_msgs::msg::BoundingBox3D> list_a, vector<vision_msgs::msg::BoundingBox3D> list_b){
  vector<vision_msgs::msg::BoundingBox3D> v;
  v.resize(list_a.size() + list_b.size());
  std::vector<vision_msgs::msg::BoundingBox3D>::iterator it;

  std::sort(list_a.begin(),list_a.end(), customCompare);
  std::sort(list_b.begin(),list_b.end(), customCompare);

  it=std::set_union(list_a.begin(), list_a.end(), list_b.begin(), list_b.end(), v.begin(), customCompare);

  v.resize(it-v.begin());
  return v;
}

void BBBenchmark::show_objects(vector<vision_msgs::msg::BoundingBox3D> objects, std::string ns){
  visualization_msgs::msg::MarkerArray msg;
  msg.markers.reserve(objects.size());
  for(unsigned long int i = 0; i<objects.size(); i++){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = _fixed_frame;
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(0.0);
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.pose = objects[i].center;
    marker.scale = objects[i].size;
    msg.markers.push_back(marker);
  }
  _debug_pub->publish(msg);
}

vector<vision_msgs::msg::BoundingBox3D> BBBenchmark::filter_camera(vector<vision_msgs::msg::BoundingBox3D> objects){
  vector<vision_msgs::msg::BoundingBox3D> on_camera;
  for(unsigned long int j=0; j<objects.size(); j++){
    tf2::Vector3 v_origin( objects[j].center.position.x,
                    objects[j].center.position.y, 
                    objects[j].center.position.z);

    for(unsigned long int id=0; id<_camera_models.size(); id++){
      if(_camera_models[id].initialized()){
        // Move the point in the tf of the camera
        tf2::Vector3 v = _tf2_transform_camera[id] * v_origin;

        // Check if point inside image
        if(v.z()<0 || v.z()>_cameras_max_dist[id])   // Object behind the camera or further then max dist
          break;
        cv::Point3d point3d(v.x(), v.y(), v.z());
        cv::Point2d point = _camera_models[id].project3dToPixel(point3d);

        if( point.x>0 && 
            point.y>0 && 
            point.x<_camera_models[id].cameraInfo().width && 
            point.y<_camera_models[id].cameraInfo().height
            ){
          on_camera.push_back(objects[j]);
          break; // Go to next object, otherwise try with the other cameras
        }
      }
    }
  }
  return on_camera;
}

vector<vision_msgs::msg::BoundingBox3D> BBBenchmark::filter_lidar(vector<vision_msgs::msg::BoundingBox3D> objects){
  vector<vision_msgs::msg::BoundingBox3D> on_lidar;
  if(!_lidar_ready){
    init_lidar_tf();
  }

  for(unsigned long int j=0; j<objects.size(); j++){
    tf2::Vector3 v_origin( objects[j].center.position.x,
                    objects[j].center.position.y, 
                    objects[j].center.position.z);
    for(unsigned long int id=0; id<_lidars.size(); id++){
      // Move the point in the tf of the camera
      tf2::Vector3 v = _tf2_transform_lidar[id] * v_origin;
      // std::cout << "vector: " 
      //           << v.x() << " , "
      //           << v.y() << " , "
      //           << v.z()
      //           << " length: " << v.length() << std::endl;
      // Check if point inside lidar_range
      if(v.length()<_lidars_max_dist[id]){
        on_lidar.push_back(objects[j]);
        break; // Go to next object, otherwise try with the other lidars
      }
    }
  }

  return on_lidar;
}

void BBBenchmark::camera_info_callback(std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_message, int id, int max_distance)
{
  geometry_msgs::msg::TransformStamped tf_result;
  try {
    tf_result = _tf_buffer.lookupTransform(cam_info_message->header.frame_id, _fixed_frame, rclcpp::Time(0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_INFO_STREAM(this->get_logger(), "No transform exists for the given tfs: " << _fixed_frame << " - " << cam_info_message->header.frame_id);
    return;
  }

  if(tf_result.transform != _last_transform_camera[id]){
    _last_transform_camera[id] = tf_result.transform;
    _camera_models[id].fromCameraInfo(cam_info_message);
    tf2::Quaternion q(
      _last_transform_camera[id].rotation.x,
      _last_transform_camera[id].rotation.y,
      _last_transform_camera[id].rotation.z,
      _last_transform_camera[id].rotation.w
    );
    tf2::Vector3 p(
      _last_transform_camera[id].translation.x,
      _last_transform_camera[id].translation.y,
      _last_transform_camera[id].translation.z
    );
    tf2::Transform transform(q, p);
    _tf2_transform_camera[id] = transform;

    if(_show_range){
      geometry_msgs::msg::Pose pose;
      shape_msgs::msg::Mesh mesh = getCameraFOVMesh(*cam_info_message.get(), max_distance);
      visualization_msgs::msg::Marker marker = getCameraFOVMarker(pose, mesh, id, cam_info_message->header.frame_id);
      _sensor_range_pub->publish(marker);
    }
  }

}

// Taken inspiration from https://github.com/ros-planning/moveit_calibration/blob/foxy/moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_context_widget.cpp
shape_msgs::msg::Mesh BBBenchmark::getCameraFOVMesh(const sensor_msgs::msg::CameraInfo& camera_info, double max_dist)
{
  shape_msgs::msg::Mesh mesh;
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);
  double delta_x = camera_model.getDeltaX(camera_info.width / 2, max_dist);
  double delta_y = camera_model.getDeltaY(camera_info.height / 2, max_dist);

  std::vector<double> x_cords = { -delta_x, delta_x };
  std::vector<double> y_cords = { -delta_y, delta_y };

  // Get corners
  mesh.vertices.clear();
  // Add the first corner at origin of the optical frame
  mesh.vertices.push_back(geometry_msgs::msg::Point());

  // Add the four corners at bottom
  for (const double& x_it : x_cords)
    for (const double& y_it : y_cords)
    {
      geometry_msgs::msg::Point vertex;
      // Check in case camera info is not valid
      if (std::isfinite(x_it) && std::isfinite(y_it) && std::isfinite(max_dist))
      {
        vertex.x = x_it;
        vertex.y = y_it;
        vertex.z = max_dist;
      }
      mesh.vertices.push_back(vertex);
    }

  // Get surface triangles
  mesh.triangles.resize(4);
  mesh.triangles[0].vertex_indices = { 0, 1, 2 };
  mesh.triangles[1].vertex_indices = { 0, 2, 4 };
  mesh.triangles[2].vertex_indices = { 0, 4, 3 };
  mesh.triangles[3].vertex_indices = { 0, 3, 1 };
  return mesh;
}

visualization_msgs::msg::Marker BBBenchmark::getCameraFOVMarker(const geometry_msgs::msg::Pose& pose,
                                                                const shape_msgs::msg::Mesh& mesh, int id, std::string frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "camera_fov";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0.0);
  marker.color.r = 255;
  marker.color.g = 255;
  marker.color.b = 0;
  marker.color.a = _alpha_range;
  marker.pose = pose;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.points.clear();
  for (const shape_msgs::msg::MeshTriangle& triangle : mesh.triangles)
    for (const uint32_t& index : triangle.vertex_indices)
      marker.points.push_back(mesh.vertices[index]);

  return marker;
}






// %%%%%%%%%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBBenchmark>());
  rclcpp::shutdown();
  return 0;
}