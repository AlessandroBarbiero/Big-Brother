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

  auto camera_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  camera_list_desc.description = "List of the camera info topics";
  auto cameras_max_dist_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cameras_max_dist_desc.description = "List of the max distances for object detected by cameras, respect the order of 'camera_list'";

  auto lidar_list_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_list_desc.description = "List of the lidar tf";
  auto lidar_max_dist_desc = rcl_interfaces::msg::ParameterDescriptor{};
  lidar_max_dist_desc.description = "List of the max distances for object detected by lidars, respect the order of 'lidar_list'";

  std::vector<std::string> cameras = {"cam1"};
  std::vector<int64_t> cameras_max_dist = {0};

  std::vector<std::string> lidars = {"lid1"};
  std::vector<int64_t> lidars_max_dist = {0};

  this->declare_parameter("fps", 30, fps_desc);
  this->declare_parameter("match_thresh", 0.8, m_thresh_desc);
  this->declare_parameter("fixed_frame", "map", fixed_frame_desc);
  this->declare_parameter("show_range", true, s_range_desc);
  this->declare_parameter("camera_list", cameras, camera_list_desc);
  this->declare_parameter("camera_max_distances", cameras_max_dist, cameras_max_dist_desc);
  this->declare_parameter("lidar_list", lidars, lidar_list_desc);
  this->declare_parameter("lidar_max_distances", lidars_max_dist, lidar_max_dist_desc);

  _fps =              get_parameter("fps").as_int();
  _match_thresh =     get_parameter("match_thresh").as_double();
  _fixed_frame =      get_parameter("fixed_frame").as_string();
  _show_range  =      get_parameter("show_range").as_bool();
  cameras =           get_parameter("camera_list").as_string_array();
  cameras_max_dist =  get_parameter("camera_max_distances").as_integer_array();
  lidars =            get_parameter("lidar_list").as_string_array();
  lidars_max_dist =   get_parameter("lidar_max_distances").as_integer_array();

  rclcpp::QoS qos_transient_local(rclcpp::KeepLast(10));
  qos_transient_local.transient_local();

  if(_show_range){
    // Add a subscriber for each camera info topic 
    int id = 0;
    for (auto camera_topic : cameras){
      rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub;
      std::function<void(std::shared_ptr<sensor_msgs::msg::CameraInfo>)> callback_fun = std::bind(
        &BBBenchmark::publish_camera_FOV, this, _1, id, cameras_max_dist[id]);
      
      cam_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_topic, rclcpp::QoS(rclcpp::KeepLast(1)), callback_fun);
      _cameras_sub.push_back(cam_sub);
      _last_transform_camera.push_back(geometry_msgs::msg::Transform());
      id++;
    }
    
    _sensor_range_pub = this->create_publisher<visualization_msgs::msg::Marker>("benchmark/sensor_range", qos_transient_local);
    
    publish_lidar_range(lidars, lidars_max_dist);
  }

  _tracker_out_sub = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "bytetrack/active_tracks", 10, std::bind(&BBBenchmark::tracker_out, this, _1));


  _static_ground_truth_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "carla/markers/static", qos_transient_local, std::bind(&BBBenchmark::save_static_gt, this, _1));
  _ground_truth_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "carla/markers", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&BBBenchmark::save_gt, this, _1));

  RCLCPP_INFO(this->get_logger(), "Benchmark Ready!");

}

void BBBenchmark::save_static_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg){
  if(msg->markers[0].header.frame_id!=_fixed_frame)
    RCLCPP_WARN(this->get_logger(), "static gt should have the same tf as fixed frame");

  _static_objects.resize(msg->markers.size());
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
  _moving_objects_bbox.resize(msg->markers.size());
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
  marker.color.a = 0.1;
  marker.pose = geometry_msgs::msg::Pose();
  marker.scale.x = range;
  marker.scale.y = range;
  marker.scale.z = range;

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

  // Convert markers in bbox only when it is necessary to compare
  save_gt_bbox(_moving_objects);

}

void BBBenchmark::publish_camera_FOV(std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_message, int id, int max_distance)
{
  geometry_msgs::msg::TransformStamped tf_result;
  try {
    tf_result = _tf_buffer.lookupTransform(_fixed_frame, cam_info_message->header.frame_id, rclcpp::Time(0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_INFO_STREAM(this->get_logger(), "No transform exists for the given tfs: " << _fixed_frame << " - " << cam_info_message->header.frame_id);
    return;
  }

  if(tf_result.transform != _last_transform_camera[id]){
    _last_transform_camera[id] = tf_result.transform;
    geometry_msgs::msg::Pose pose;
    shape_msgs::msg::Mesh mesh = getCameraFOVMesh(*cam_info_message.get(), max_distance);
    visualization_msgs::msg::Marker marker = getCameraFOVMarker(pose, mesh, id, cam_info_message->header.frame_id);

    _sensor_range_pub->publish(marker);
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
  marker.color.a = 0.1;
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