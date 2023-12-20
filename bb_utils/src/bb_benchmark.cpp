#include <bb_utils/bb_benchmark.hpp>
#include <bb_utils/lapjv.h>
#define HIDE_DEBUG_MESSAGES

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

  auto selection_desc = rcl_interfaces::msg::ParameterDescriptor{};
  selection_desc.description = "Set to true if a gt object is currently being tracked and want to publish the relative data";

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

  this->declare_parameter("active_selection", false, selection_desc);

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
      "bytetrack/active_tracks", 10, std::bind(&BBBenchmark::compute_stats, this, _1));

  _static_ground_truth_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "carla/markers/static", qos_transient_local, std::bind(&BBBenchmark::save_static_gt, this, _1));
  _ground_truth_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "carla/markers", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&BBBenchmark::save_gt, this, _1));

  _clicked_point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 10, std::bind(&BBBenchmark::point_clicked, this, _1));


  _debug_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("benchmark/debug", 10);
  _stats_pub = this->create_publisher<bb_interfaces::msg::Stats>("benchmark/stats", 10);
  _selected_gt_pub = this->create_publisher<visualization_msgs::msg::Marker>("benchmark/selected_gt", 10);
  _selected_track_pub = this->create_publisher<visualization_msgs::msg::Marker>("benchmark/selected_track", 10);

  _gt_file.open(_gt_file_name, std::ios_base::out | std::ios::trunc);
  if (!_gt_file.is_open()) {
    std::cerr << "Unable to open file " << _gt_file_name << " for appending!" << std::endl;
  }
  _track_file.open(_track_file_name, std::ios_base::out | std::ios::trunc);
  if (!_track_file.is_open()) {
    std::cerr << "Unable to open file " << _track_file_name << " for appending!" << std::endl;
  }
  std::string header = "timestamp,detection_number,id,class,x,y,z,theta,size_x,size_y,size_z";
  _gt_file << header << std::endl;
  _track_file << header << std::endl;

  RCLCPP_INFO(this->get_logger(), "Benchmark Ready!");

}

BBBenchmark::~BBBenchmark(){
  _gt_file.close();
  _track_file.close();
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

bb_bench::ClassLabel get_label(std::string ns, double size_x, double size_y){
  std::string class_name;
  class_name = ns;
    if(class_name.empty()){
      if (size_x>2.5 || size_y>2.5)
          class_name = "car";
      else if (size_x>1.0 || size_y>1.0)
          class_name = "motorcycle";
      else
          class_name = "person";
    }else{
      // lowercase string
      std::transform(class_name.begin(), class_name.end(), class_name.begin(),
        [](unsigned char c){ return std::tolower(c); });
    }
    return bb_bench::class_to_label.at(class_name);
}

void BBBenchmark::save_static_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg){
  if(msg->markers[0].header.frame_id!=_fixed_frame)
    RCLCPP_WARN(this->get_logger(), "static gt should have the same tf as fixed frame");

  _static_objects.reserve(msg->markers.size());

  for(auto marker: msg->markers){
    vision_msgs::msg::BoundingBox3D bbox;
    bbox.center=marker.pose;
    bbox.size=marker.scale;

    bb_bench::Object3D obj = {marker.id, get_label(marker.ns, marker.scale.x, marker.scale.y), bbox};
    _static_objects.push_back(obj);
  }

  RCLCPP_INFO(this->get_logger(), "Static Objects saved!");
}

void BBBenchmark::save_gt(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg)
{
  _moving_objects = msg;
}

void BBBenchmark::save_gt_bbox(std::shared_ptr<visualization_msgs::msg::MarkerArray> msg)
{
  if(msg.get()==nullptr)
    return;
  _moving_objects_bbox.clear();
  _moving_objects_bbox.reserve(msg->markers.size());
  for(auto marker: msg->markers){
    vision_msgs::msg::BoundingBox3D bbox;
    bbox.center=marker.pose;
    bbox.size=marker.scale;

    bb_bench::Object3D obj = {marker.id, get_label(marker.ns, marker.scale.x, marker.scale.y), bbox};
    _moving_objects_bbox.push_back(obj);
  }

  // RCLCPP_INFO_STREAM(this->get_logger(), "Moving Objects saved! " << _moving_objects_bbox.size() << " Objects");
}

int binarySearch_id(vector<bb_bench::Object3D> &objects, int id) {
  int high,low,mid;
  high = objects.size() - 1;
  low = 0;
  while (low <= high) {
    mid = low + (high - low) / 2;

    if (id == objects[mid].id)
      return mid;

    if (id > objects[mid].id)
      low = mid + 1;

    if (id < objects[mid].id)
      high = mid - 1;
  }

  return -1;
}

int binarySearch_id(std::vector<vision_msgs::msg::Detection3D> &objects, std::string track_id) {
  int high,low,mid;
  high = objects.size() - 1;
  low = 0;
  while (low <= high) {
    mid = low + (high - low) / 2;

    if (track_id == objects[mid].tracking_id)
      return mid;

    if (track_id > objects[mid].tracking_id)
      low = mid + 1;

    if (track_id < objects[mid].tracking_id)
      high = mid - 1;
  }

  return -1;
}

void BBBenchmark::publish_selected_obj(std::vector<vision_msgs::msg::Detection3D>& tracked_obj){
  if(!get_parameter("active_selection").as_bool())
    return;

  // Find element with id equal to saved and publish
  std::sort(_all_objects.begin(),_all_objects.end(), [](const bb_bench::Object3D &a, const bb_bench::Object3D &b) {
    return a.id < b.id;
    }
  );
  int gt_index = binarySearch_id(_all_objects, _selected_gt_id);
  if(gt_index == -1)
    return; // No object with the selected index, probably deleted 
  auto gt_object = _all_objects[gt_index];

  visualization_msgs::msg::Marker marker_gt;
    marker_gt.header.frame_id = _fixed_frame;
    marker_gt.header.stamp = tracked_obj[0].header.stamp;
    marker_gt.ns = bb_bench::classLabelString[static_cast<int>(gt_object.label)];
    marker_gt.id = gt_index;
    marker_gt.type = visualization_msgs::msg::Marker::CUBE;
    marker_gt.action = visualization_msgs::msg::Marker::ADD;
    marker_gt.lifetime = rclcpp::Duration(0,200*10e6);
    marker_gt.color.r = 0;
    marker_gt.color.g = 255;
    marker_gt.color.b = 255;
    marker_gt.color.a = 0.9;
    marker_gt.pose = gt_object.bbox.center;
    marker_gt.scale = gt_object.bbox.size;
  _selected_gt_pub->publish(marker_gt);
  //---------------

  // Find element with track id correspondent and publish
  auto it = _gt_index_to_track_id.find(_selected_gt_id);
  if(it == _gt_index_to_track_id.end())
    return; // No tracked object associated

  std::string track_id = it->second;
  std::sort(tracked_obj.begin(),tracked_obj.end(), [](const vision_msgs::msg::Detection3D &a, const vision_msgs::msg::Detection3D &b) {
    return a.tracking_id < b.tracking_id;
    }
  );
  int track_index = binarySearch_id(tracked_obj, track_id);
  if(track_index == -1)
    return; // No tracked object with the selected index, probably deleted 
  auto track_object = tracked_obj[track_index];

  visualization_msgs::msg::Marker marker_track;
    marker_track.header.frame_id = _fixed_frame;
    marker_track.header.stamp = track_object.header.stamp;
    marker_track.ns = "TrackedObject_selected";
    marker_track.id = atoi(track_id.c_str());
    marker_track.type = visualization_msgs::msg::Marker::CUBE;
    marker_track.action = visualization_msgs::msg::Marker::ADD;
    marker_track.lifetime = rclcpp::Duration(0,200*10e6);
    marker_track.color.r = 255;
    marker_track.color.g = 0;
    marker_track.color.b = 255;
    marker_track.color.a = 0.9;
    marker_track.pose = track_object.bbox.center;
    marker_track.scale = track_object.bbox.size;
  _selected_track_pub->publish(marker_track);

  static std::string last_track_id = "";
  if(last_track_id != track_id){
    RCLCPP_INFO_STREAM(this->get_logger(), "Changed Track id! Tracking object id = " << track_id);
    last_track_id = track_id;
  }

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

void BBBenchmark::change_frame(std::shared_ptr<geometry_msgs::msg::PointStamped> old_message, std::string& new_frame){
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

  tf2::Vector3 v(old_message->point.x, old_message->point.y, old_message->point.z);
  v = transform * v;
  old_message->point.x = v.x();
  old_message->point.y = v.y();
  old_message->point.z = v.z();

  old_message->header.frame_id = new_frame;
  return;
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

// %%%%%%%%%%%%%%%%%%%%%%% Linear assignment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

void BBBenchmark::linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
	vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b)
{
	if (cost_matrix.size() == 0)
	{
		for (int i = 0; i < cost_matrix_size; i++)
		{
			unmatched_a.push_back(i);
		}
		for (int i = 0; i < cost_matrix_size_size; i++)
		{
			unmatched_b.push_back(i);
		}
		return;
	}

	vector<int> rowsol; vector<int> colsol;
	//unused variable
	//float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
	lapjv(cost_matrix, rowsol, colsol, true, thresh);
	for (unsigned int i = 0; i < rowsol.size(); i++)
	{
		if (rowsol[i] >= 0)
		{
			vector<int> match;
			match.push_back(i);
			match.push_back(rowsol[i]);
			matches.push_back(match);
		}
		else
		{
			unmatched_a.push_back(i);
		}
	}

	for (unsigned int i = 0; i < colsol.size(); i++)
	{
		if (colsol[i] < 0)
		{
			unmatched_b.push_back(i);
		}
	}
}

double BBBenchmark::lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
	bool extend_cost, float cost_limit, bool return_cost)
{
	vector<vector<float> > cost_c;
	cost_c.assign(cost.begin(), cost.end());

	vector<vector<float> > cost_c_extended;

	int n_rows = cost.size();
	int n_cols = cost[0].size();
	rowsol.resize(n_rows);
	colsol.resize(n_cols);

	int n = 0;
	if (n_rows == n_cols)
	{
		n = n_rows;
	}
	else
	{
		if (!extend_cost)
		{
			cout << "set extend_cost=True" << endl;
			system("pause");
			exit(0);
		}
	}
		
	if (extend_cost || cost_limit < LONG_MAX)
	{
		n = n_rows + n_cols;
		cost_c_extended.resize(n);
		for (unsigned int i = 0; i < cost_c_extended.size(); i++)
			cost_c_extended[i].resize(n);

		if (cost_limit < LONG_MAX)
		{
			for (unsigned int i = 0; i < cost_c_extended.size(); i++)
			{
				for (unsigned int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_limit / 2.0;
				}
			}
		}
		else
		{
			float cost_max = -1;
			for (unsigned int i = 0; i < cost_c.size(); i++)
			{
				for (unsigned int j = 0; j < cost_c[i].size(); j++)
				{
					if (cost_c[i][j] > cost_max)
						cost_max = cost_c[i][j];
				}
			}
			for (unsigned int i = 0; i < cost_c_extended.size(); i++)
			{
				for (unsigned int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_max + 1;
				}
			}
		}

		for (unsigned int i = n_rows; i < cost_c_extended.size(); i++)
		{
			for (unsigned int j = n_cols; j < cost_c_extended[i].size(); j++)
			{
				cost_c_extended[i][j] = 0;
			}
		}
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				cost_c_extended[i][j] = cost_c[i][j];
			}
		}

		cost_c.clear();
		cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
	}

	double **cost_ptr;
	cost_ptr = new double *[sizeof(double *) * n];
	for (int i = 0; i < n; i++)
		cost_ptr[i] = new double[sizeof(double) * n];

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cost_ptr[i][j] = cost_c[i][j];
		}
	}

	int* x_c = new int[sizeof(int) * n];
	int *y_c = new int[sizeof(int) * n];

	int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
	if (ret != 0)
	{
		cout << "Calculate Wrong!" << endl;
		system("pause");
		exit(0);
	}

	double opt = 0.0;

	if (n != n_rows)
	{
		for (int i = 0; i < n; i++)
		{
			if (x_c[i] >= n_cols)
				x_c[i] = -1;
			if (y_c[i] >= n_rows)
				y_c[i] = -1;
		}
		for (int i = 0; i < n_rows; i++)
		{
			rowsol[i] = x_c[i];
		}
		for (int i = 0; i < n_cols; i++)
		{
			colsol[i] = y_c[i];
		}

		if (return_cost)
		{
			for (unsigned int i = 0; i < rowsol.size(); i++)
			{
				if (rowsol[i] != -1)
				{
					//cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
					opt += cost_ptr[i][rowsol[i]];
				}
			}
		}
	}
	else if (return_cost)
	{
		for (unsigned int i = 0; i < rowsol.size(); i++)
		{
			opt += cost_ptr[i][rowsol[i]];
		}
	}

	for (int i = 0; i < n; i++)
	{
		delete[]cost_ptr[i];
	}
	delete[]cost_ptr;
	delete[]x_c;
	delete[]y_c;

	return opt;
}

vector<float> to_minmax(vision_msgs::msg::BoundingBox3D &bbox){
  vector<float> minmax;
  minmax.resize(6);
  minmax[0] = bbox.center.position.x - bbox.size.x/2;
  minmax[1] = bbox.center.position.y - bbox.size.y/2;
  minmax[2] = bbox.center.position.z - bbox.size.z/2;
  minmax[3] = bbox.center.position.x + bbox.size.x/2;
  minmax[4] = bbox.center.position.y + bbox.size.y/2;
  minmax[5] = bbox.center.position.z + bbox.size.z/2;
  return minmax;
}

vector<float> to_minmax(vision_msgs::msg::Detection3D &det){
  vector<float> minmax;
  vision_msgs::msg::BoundingBox3D bbox = det.bbox;
  minmax.resize(6);
  minmax[0] = bbox.center.position.x - bbox.size.x/2;
  minmax[1] = bbox.center.position.y - bbox.size.y/2;
  minmax[2] = bbox.center.position.z - bbox.size.z/2;
  minmax[3] = bbox.center.position.x + bbox.size.x/2;
  minmax[4] = bbox.center.position.y + bbox.size.y/2;
  minmax[5] = bbox.center.position.z + bbox.size.z/2;
  return minmax;
}

vector<vector<float> > BBBenchmark::iou_distance(vector<bb_bench::Object3D> &a_bboxs, vector<vision_msgs::msg::Detection3D> &b_bboxs)
{
	vector<vector<float> > aminmaxs, bminmaxs;
	for (unsigned int i = 0; i < a_bboxs.size(); i++)
	{
		aminmaxs.push_back(to_minmax(a_bboxs[i].bbox));
	}
	for (unsigned int i = 0; i < b_bboxs.size(); i++)
	{
		bminmaxs.push_back(to_minmax(b_bboxs[i].bbox));
	}

	vector<vector<float> > _ious = ious(aminmaxs, bminmaxs);
	vector<vector<float> > cost_matrix;
	for (unsigned int i = 0; i < _ious.size(); i++)
	{
		vector<float> _iou;
		for (unsigned int j = 0; j < _ious[i].size(); j++)
		{
			_iou.push_back(1 - _ious[i][j]);
		}
		cost_matrix.push_back(_iou);
	}

	return cost_matrix;
}

// -------------------------------------------------------

template<typename T>
vector<T> filter_indices(vector<T> &source_vector, vector<int> &indices){
  vector<T> target_vector;
  target_vector.reserve(indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    // Not handling index out of range
    target_vector.push_back(source_vector[indices[i]]);
  }
  return target_vector;
}

// %%%%%%%%%%%%%%%%%% main part %%%%%%%%%%%%%%%

void BBBenchmark::compute_stats(std::shared_ptr<vision_msgs::msg::Detection3DArray> tracked_objects)
{
  static int detection_number = 0;
  int objects_to_detect, false_positive, true_positive, missed;
  float tot_iou = 0, tot_dist = 0;
  double detA, locA, MOTP, tot_detA = 0, tot_locA = 0, tot_MOTP = 1, MOTA;
  // Convert markers of moving objects in bbox only when it is necessary to compare
  save_gt_bbox(_moving_objects);
  // Put moving and static objects together
  _all_objects.clear();
  _all_objects.reserve(_static_objects.size()+_moving_objects_bbox.size());
  _all_objects.insert(_all_objects.end(), _static_objects.begin(), _static_objects.end());
  _all_objects.insert(_all_objects.end(), _moving_objects_bbox.begin(), _moving_objects_bbox.end());

  // Retrive objects on cameras and lidars
  vector<bb_bench::Object3D> on_camera = filter_camera(_all_objects);
  vector<bb_bench::Object3D> on_lidar = filter_lidar(_all_objects);
  vector<bb_bench::Object3D> objects_on_sight = obj_union(on_camera, on_lidar);

  // RCLCPP_INFO_STREAM(this->get_logger(), "Objects on sight: " << objects_on_sight.size() << " [by Cameras: " << on_camera.size() << " - by Lidars: " << on_lidar.size() << "]");
  
  show_objects(on_camera, "Objects_on_camera");
  show_objects(on_lidar, "Objects_on_lidar");
  show_objects(objects_on_sight, "Objects_in_sensor_range");
  objects_to_detect = objects_on_sight.size();

  // %%%%%%%%%%%% Write CSV

  std::string timestamp_str = std::to_string(tracked_objects->header.stamp.nanosec + static_cast<int64_t>(tracked_objects->header.stamp.sec) * NANOSEC_IN_SEC);
  for(auto obj : objects_on_sight){
    _gt_file << timestamp_str << "," << detection_number << "," << obj.toCSV() << std::endl;
  }

  for(auto det : tracked_objects->detections){
    std::string class_name = det.results[0].id;
    std::transform(class_name.begin(), class_name.end(), class_name.begin(),
        [](unsigned char c){ return std::tolower(c); });

    bb_bench::Object3D temp = {atoi(det.tracking_id.c_str()), bb_bench::class_to_label.at(class_name), det.bbox};
    _track_file << timestamp_str << "," << detection_number << "," << temp.toCSV() << std::endl;
  }

  // %%%%%%%%%%%%%%%%%%%%%

  //TODO: HOTA (Higher Order Tracking Accuracy) it uses 3 accuracy scores: locA, detA, assA.
  // locA -> Localization Accuracy (LocA) by averaging the Loc-IoU over all pairs of matching predicted and ground-truth detections (if there is a match how much they intersect)
  // MOTP -> (multiple object tracking precision) uses distance instead of accuracy, low values are better MOTP = sum(d_t)/sum(matches_t)   for example d_t = 1-IoU
  // detA -> Detection Accuracy (DetA) computed as DetA = TP / (TP+FP+m) True Positive = matching detections, False Positive = predicted detections that don't match, Misses = Ground Truth detection that don't match
  // TODO: assA -> Association Accuracy (AssA). AssA = average_of( TPA / (TPA+FPA+FNA) ) for all the matched detections. 
  //          TPA = number of matching detections with ground truth for that track in time. FPA = predicted trajectory not true. FNA = real trajectory not matched.
  // HOTA_alpha = sqrt(DetA_alpha * AssA_alpha) where alpha is the value of the hungarian algorithm
  // HOTA = 1/19 * sum(HOTA_alpa)_alpha from 0.05 to 1 increasing by 0.05

  // MOTA = 1 - sum(m_t + FP_t + mme_t)/sum(obj_in_scene_t)     m = misses, FP = false positive, mme = mismatches (tracks are exchanged, object considered in wrong track)

  if (tracked_objects->detections.empty()){
    true_positive = 0;
    missed = objects_to_detect;
    false_positive = 0;
    detA = 0;
    locA = 0;
    MOTP = 1;
  }
  else{
    // Move all tracked and gt to a common fixed frame
    if(tracked_objects->header.frame_id != _fixed_frame){
      change_frame(tracked_objects, _fixed_frame);
    }

    // Find matches with hungarian algorithm
    vector<vector<float> > dists;
    dists = iou_distance(objects_on_sight, tracked_objects->detections);
    vector<vector<int> > matches;
    vector<int> missed_obj, wrong_tracked;
    // Here I am using the same threshold for car and pedestrians, consider using a lower threshold for pedestrians
    linear_assignment(dists, dists.size(), dists[0].size(), _match_thresh, matches, missed_obj, wrong_tracked);
    true_positive = matches.size();
    false_positive = wrong_tracked.size();
    missed = missed_obj.size();

    vector<int> match_gt;
    vector<int> match_track;
    for(auto match:matches){
      match_gt.push_back(match[0]);
      match_track.push_back(match[1]);
    }

    // ---Compute mismatch
    // Save at each match the corrispondence for a given gt id -> at next match if the id is different there is an association mismatch
    int gt_index;
    std::string track_id;
    for(size_t i=0; i<match_gt.size(); i++){
      gt_index = objects_on_sight[match_gt[i]].id;
      track_id = tracked_objects->detections[match_track[i]].tracking_id;
      if(_gt_index_to_track_id.count(gt_index) != 0 && _gt_index_to_track_id[gt_index] != track_id)
        _tot_ass_mismatch++;
      _gt_index_to_track_id[gt_index] = track_id; 
    }
    //------

    vector<bb_bench::Object3D> true_positive_gt =                 filter_indices(objects_on_sight, match_gt);
    vector<vision_msgs::msg::Detection3D> true_positive_track =   filter_indices(tracked_objects->detections, match_track);
    vector<vision_msgs::msg::Detection3D> false_positive_det =    filter_indices(tracked_objects->detections, wrong_tracked);
    vector<bb_bench::Object3D> missed_gt =                        filter_indices(objects_on_sight, missed_obj);
    show_objects(true_positive_gt, "True Positive");
    show_objects(missed_gt, "Missed Objects");
    
    for(auto match:matches){
      tot_iou += 1 - dists[match[0]][match[1]];
      tot_dist += dists[match[0]][match[1]];
    }
    if(true_positive == 0){
      locA = 0;
      detA = 0;
      MOTP = 1;
      MOTA = 0;
    }else{
      locA = tot_iou/true_positive;
      detA = static_cast<double>(true_positive) / (true_positive + false_positive + missed);
      MOTP = tot_dist/true_positive;
    }

  }

  _tot_iou_detections+=     tot_iou;
  _tot_iou_dist_detections+=tot_dist;

  _tot_true_positive+=      true_positive;
  _tot_false_positive+=     false_positive;
  _tot_missed+=             missed;
  _tot_objects_to_detect+=  objects_to_detect;

  if(_tot_true_positive != 0){
    tot_locA = _tot_iou_detections/_tot_true_positive;
    tot_detA = static_cast<double>(_tot_true_positive) / (_tot_true_positive + _tot_false_positive + _tot_missed);
    tot_MOTP = _tot_iou_dist_detections/_tot_true_positive;
  }

  MOTA = 1.0 - static_cast<double>(_tot_missed + _tot_false_positive + _tot_ass_mismatch)/_tot_objects_to_detect;

  // RCLCPP_INFO_STREAM(this->get_logger(), "Stats: \n" << 
  //                                         "\tDetA: " << detA*100 << "%\n" << 
  //                                         "\tLocA: " << locA*100 << "%\n" <<
  //                                         "\tMOTP: " << MOTP << "\n" <<
  //                                         "\tTotal DetA: " << tot_detA*100 << "%\n" << 
  //                                         "\tTotal LocA: " << tot_locA*100 << "%\n" <<
  //                                         "\tTotal MOTP: " << tot_MOTP
  //                                         );

  bb_interfaces::msg::Stats stats_message;
  stats_message.det_a =                 detA;
  stats_message.loc_a =                 locA;
  stats_message.motp =                  MOTP;
  stats_message.true_positive =         true_positive;
  stats_message.false_positive =        false_positive;
  stats_message.missed =                missed;
  stats_message.objects_to_detect =     objects_to_detect;
  stats_message.tot_det_a =               tot_detA;
  stats_message.tot_loc_a =               tot_locA;
  stats_message.tot_motp =                tot_MOTP;
  stats_message.tot_true_positive =       _tot_true_positive;
  stats_message.tot_false_positive =      _tot_false_positive;
  stats_message.tot_missed =              _tot_missed;
  stats_message.tot_objects_to_detect =   _tot_objects_to_detect;
  stats_message.tot_ass_mismatch =        _tot_ass_mismatch;
  stats_message.mota =                    MOTA;

  
  _stats_pub->publish(stats_message);
  publish_selected_obj(tracked_objects->detections);
  detection_number++;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Custom comparison function for BoundingBoxes
bool customCompare(const bb_bench::Object3D& obj1, const bb_bench::Object3D& obj2) {
  if (obj1.bbox.center.position.x != obj2.bbox.center.position.x) return obj1.bbox.center.position.x < obj2.bbox.center.position.x;
  if (obj1.bbox.center.position.y != obj2.bbox.center.position.y) return obj1.bbox.center.position.y < obj2.bbox.center.position.y;
  return obj1.bbox.center.position.z < obj2.bbox.center.position.z;

}

int binarySearch_nearX(vector<bb_bench::Object3D>& objects, double x_value) {
  int high,low,mid;
  high = objects.size() - 1;
  low = 0;
  double distance;
  while (low <= high) {
    mid = low + (high - low) / 2;
    distance = x_value - objects[mid].bbox.center.position.x;

    if (std::abs(distance) < 3)
      return mid;

    if (distance > 0)
      low = mid + 1;

    if (distance < 0)
      high = mid - 1;
  }

  return -1;
}

double distanceBetweenPoints(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}


void BBBenchmark::point_clicked(std::shared_ptr<geometry_msgs::msg::PointStamped> cp_message){
  static const float searching_distance = 5.0;

  if(_fixed_frame != cp_message->header.frame_id){
    change_frame(cp_message, _fixed_frame);
  }

  geometry_msgs::msg::Point clicked_point = cp_message->point;

  std::sort(_all_objects.begin(),_all_objects.end(), customCompare);

  // Find element with binary search close to the x coordinate 
  int index = binarySearch_nearX(_all_objects, cp_message->point.x);
  if(index==-1)
    return;

  // Search around that point on objects with x within +- 5 meters the one with min distance
  double min_distance = 1000;
  _selected_gt_id = _all_objects[index].id;
  double dist;
  for(int i=index; i<static_cast<int>(_all_objects.size()) && std::abs(_all_objects[i].bbox.center.position.x - cp_message->point.x) < searching_distance; i++){
    dist = distanceBetweenPoints(clicked_point, _all_objects[i].bbox.center.position);
    if(dist < min_distance){
      min_distance = dist;
      _selected_gt_id = _all_objects[i].id;
    }
  }
  for(int i=index; i>0 && std::abs(_all_objects[i].bbox.center.position.x - cp_message->point.x) < searching_distance; i--){
    dist = distanceBetweenPoints(clicked_point, _all_objects[i].bbox.center.position);
    if(dist < min_distance){
      min_distance = dist;
      _selected_gt_id = _all_objects[i].id;
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Start following gt_object number " << _selected_gt_id);

  rclcpp::Parameter param("active_selection", true);
  this->set_parameter(param);
}

vector<bb_bench::Object3D> BBBenchmark::obj_union(vector<bb_bench::Object3D>& list_a, vector<bb_bench::Object3D>& list_b){
  vector<bb_bench::Object3D> v;
  v.resize(list_a.size() + list_b.size());
  std::vector<bb_bench::Object3D>::iterator it;

  std::sort(list_a.begin(),list_a.end(), customCompare);
  std::sort(list_b.begin(),list_b.end(), customCompare);

  it=std::set_union(list_a.begin(), list_a.end(), list_b.begin(), list_b.end(), v.begin(), customCompare);

  v.resize(it-v.begin());
  return v;
}

void BBBenchmark::show_objects(vector<bb_bench::Object3D> objects, std::string ns){
  #ifndef HIDE_DEBUG_MESSAGES
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
    marker.pose = objects[i].bbox.center;
    marker.scale = objects[i].bbox.size;
    msg.markers.push_back(marker);
  }
  _debug_pub->publish(msg);
  #endif
}

vector<bb_bench::Object3D> BBBenchmark::filter_camera(vector<bb_bench::Object3D>& objects){
  vector<bb_bench::Object3D> on_camera;
  for(unsigned long int j=0; j<objects.size(); j++){
    tf2::Vector3 v_origin( objects[j].bbox.center.position.x,
                    objects[j].bbox.center.position.y, 
                    objects[j].bbox.center.position.z);

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

vector<bb_bench::Object3D> BBBenchmark::filter_lidar(vector<bb_bench::Object3D>& objects){
  vector<bb_bench::Object3D> on_lidar;
  if(!_lidar_ready){
    init_lidar_tf();
  }

  for(unsigned long int j=0; j<objects.size(); j++){
    tf2::Vector3 v_origin( objects[j].bbox.center.position.x,
                    objects[j].bbox.center.position.y, 
                    objects[j].bbox.center.position.z);
    for(unsigned long int id=0; id<_lidars.size(); id++){
      // Move the point in the tf of the lidar
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