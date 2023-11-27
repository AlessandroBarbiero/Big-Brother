// C++
#include <iostream>
#include <iomanip>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <chrono>
#include <regex>
// ROS
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/serialization.hpp"
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
// ROS messages
#include <visualization_msgs/msg/marker_array.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <rcl_interfaces/msg/log.hpp>

#define NANOSEC_IN_SEC int64_t(1000000000)
#define SEC_IN_HOUR int64_t(3600)

// To run this type:
// ros2 run bb_utils rewrite_bag_timestamps /path/to/input.bag /path/to/output.bag [-a]
// Example:
// ros2 run bb_utils rewrite_bag_timestamps /home/ale/bag_files/static_sensors_only /home/ale/bag_files/static_sensors_only_correct_ts -a

builtin_interfaces::msg::Time getStamp(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message, std::string topic_type);
bool endsWithSubstring(const std::string& str, const std::string& subStr);

// Function that rewrites the timestamps inside the input_bag to match the timestamps inside the messages and publishes a new bag
void rewriteBagTimestamps(const std::string& input_bag, const std::string& output_bag, bool keep_all_topics = false)
{

  std::vector<std::string> topics_to_keep = {
                                    "/carla/sensors_home/debug_camera/camera_info",
                                    "/carla/sensors_home/debug_camera/image",
                                    "/carla/markers",
                                    "/carla/markers/static",
                                    "/carla/objects",
                                    "/carla/sensors_home/static_rgb_camera/image", 
                                    "/carla/sensors_home/static_depth_camera/image",
                                    "/carla/sensors_home/static_termic_camera/image",
                                    "/carla/sensors_home/static_rgb_camera/camera_info", 
                                    "/carla/sensors_home/static_depth_camera/camera_info",
                                    "/carla/sensors_home/static_termic_camera/camera_info",
                                    "/carla/sensors_home/static_lidar",
                                    "/tf",
                                    "/tf_static",
                                    "/clock"
                                    };
  
  std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer;

  // Configure the storage options for the reader and writer
  const rosbag2_cpp::StorageOptions r_storage_options({input_bag, "sqlite3"});
  const rosbag2_cpp::StorageOptions w_storage_options({output_bag, "sqlite3"});
  const rosbag2_cpp::ConverterOptions converter_options(
    {rmw_get_serialization_format(),
      rmw_get_serialization_format()});

  // Open the input bag for reading
  reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  reader->open(r_storage_options, converter_options);

  // Open the output bag for writing
  writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  writer->open(w_storage_options, converter_options);
  
  // Set the filter
  rosbag2_storage::StorageFilter storage_filter;

  if(keep_all_topics){
    const auto all_topics = reader->get_all_topics_and_types();
    topics_to_keep.clear();
    for (auto topic : all_topics) {
      topics_to_keep.push_back(topic.name);
    }
  }

  for (auto topic : topics_to_keep) {
      storage_filter.topics.push_back(topic);
  }
  reader->set_filter(storage_filter);

  // Recreate the topics in the new bag
  std::unordered_map<std::string, std::string> topic_to_type;
  const auto topics = reader->get_all_topics_and_types();
  for (const auto topic : topics){

    auto iter = std::find(topics_to_keep.begin(), topics_to_keep.end(), topic.name);
    // If the topic is in the topics to keep
    if(iter != topics_to_keep.end()){ 
      topic_to_type.emplace(topic.name, topic.type);
      writer->create_topic(
        {topic.name,
        topic.type,
        rmw_get_serialization_format(),
        topic.offered_qos_profiles});
    }
  }

  int tot_messages = reader->get_metadata().message_count;
  int counter = 0;

  // Iterate over messages in the input bag
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;
  std::string topic_type;
  builtin_interfaces::msg::Time stamp;
  std::cout<< "Start reading the bag... \n\t"<< tot_messages << " Messages to convert"<<std::endl;

  auto start_time = std::chrono::high_resolution_clock::now();
  std::cout << std::fixed << std::setprecision(2);

  int64_t timestamp;
  // The starting time of the messages should be the same as the bag to convert
  auto start_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(reader->get_metadata().starting_time);
  int64_t bag_start_time = start_ns.time_since_epoch().count();

  int64_t last_clock = 0;
  int64_t first_clock = 0;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_to_add;
  std::string name;
  while (rclcpp::ok() && reader->has_next())
  {
    // Read the next message
    bag_message = reader->read_next();

    if(endsWithSubstring(bag_message->topic_name, "static")){
      if(last_clock == 0){ // Never arrived a clock message
        messages_to_add.push_back(bag_message);
        continue;
      }
      else
        timestamp = last_clock;
    }
    else{
      topic_type = topic_to_type.at(bag_message->topic_name);
      // Get the timestamp from the header
      stamp = getStamp(bag_message, topic_type);
      timestamp = int64_t(stamp.sec) * NANOSEC_IN_SEC + int64_t(stamp.nanosec);

      if(bag_message->topic_name=="/clock"){
        last_clock = timestamp;
        if(first_clock == 0)
          first_clock = timestamp;
        // Recover static messages
        if(messages_to_add.size()!=0){
          for(auto msg : messages_to_add){
            msg->time_stamp = bag_start_time + (last_clock - first_clock);
            writer->write(msg);
            counter++;
          }
          messages_to_add.clear();
        }
      }
    }

    // std::cout << "topic name: \t" << bag_message->topic_name << std::endl;
    // std::cout << "stamp: \t\t" << bag_message->time_stamp << std::endl;
    // std::cout << "header stamp: \t" << timestamp << std::endl;

    // if the timestamp is before the starting time of the bag -> bring everything to the correct time
    if (timestamp < bag_start_time)
      timestamp = bag_start_time + (timestamp - first_clock);

    // std::cout << "published header stamp: \t" << timestamp << std::endl;

    // Update the message timestamp
    bag_message->time_stamp=timestamp;

    // Write the modified message to the output bag
    writer->write(bag_message);
    counter++;
    std::cout << "[ " <<counter*100.0/tot_messages << "% ]\r";
  }

  std::cout<< "Finished" << std::endl;
  // Stop the clock
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start_time;
  // Print the duration in seconds
  std::cout << "Execution time: " << duration.count() << " seconds." << std::endl;
}

bool endsWithSubstring(
            const std::string& str,
            const std::string& subStr) {
    // Regular expression pattern for checking if string ends with the substring
    const std::regex pattern(subStr + R"($)");
    // Check if the string matches the pattern
    return std::regex_search(str, pattern);
}

builtin_interfaces::msg::Time getStamp(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message, std::string topic_type){

  static rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
  static rclcpp::Serialization<sensor_msgs::msg::CameraInfo> cam_info_serializer;
  static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> lidar_serializer;
  static rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer;
  static rclcpp::Serialization<rosgraph_msgs::msg::Clock> clock_serializer;
  static rclcpp::Serialization<visualization_msgs::msg::MarkerArray> marker_array_serializer;
  static rclcpp::Serialization<derived_object_msgs::msg::ObjectArray> object_array_serializer;
  static rclcpp::Serialization<vision_msgs::msg::Detection2DArray> det2d_serializer;
  static rclcpp::Serialization<vision_msgs::msg::Detection3DArray> det3d_serializer;
  static rclcpp::Serialization<rcl_interfaces::msg::Log> log_serializer;

  builtin_interfaces::msg::Time stamp;
  rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    if(topic_type == "sensor_msgs/msg/Image"){
      sensor_msgs::msg::Image extracted_msg;
      image_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.header.stamp;
    }
    else if(topic_type == "sensor_msgs/msg/PointCloud2"){
      sensor_msgs::msg::PointCloud2 extracted_msg;
      lidar_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.header.stamp;
    }
    else if(topic_type == "tf2_msgs/msg/TFMessage"){
      tf2_msgs::msg::TFMessage extracted_msg;
      tf_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.transforms[0].header.stamp;
    }
    else if(topic_type == "rosgraph_msgs/msg/Clock"){
      rosgraph_msgs::msg::Clock extracted_msg;
      clock_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.clock;
    }
    else if(topic_type == "sensor_msgs/msg/CameraInfo"){
      sensor_msgs::msg::CameraInfo extracted_msg;
      cam_info_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.header.stamp;
    }
    else if(topic_type == "visualization_msgs/msg/MarkerArray"){
      visualization_msgs::msg::MarkerArray extracted_msg;
      marker_array_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.markers[0].header.stamp;
    }
    else if(topic_type == "derived_object_msgs/msg/ObjectArray"){
      derived_object_msgs::msg::ObjectArray extracted_msg;
      object_array_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.header.stamp;
    }
    else if(topic_type == "vision_msgs/msg/Detection2DArray"){
      vision_msgs::msg::Detection2DArray extracted_msg;
      det2d_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.header.stamp;
    }
    else if(topic_type == "vision_msgs/msg/Detection3DArray"){
      vision_msgs::msg::Detection3DArray extracted_msg;
      det3d_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.header.stamp;
    }
    else if(topic_type == "rcl_interfaces/msg/Log"){
      rcl_interfaces::msg::Log extracted_msg;
      log_serializer.deserialize_message(&extracted_serialized_msg, &extracted_msg);
      stamp = extracted_msg.stamp;
    }
    else{
      std::cerr << "Type " << topic_type << " not supported, look at getStamp() function" << std::endl;
    }

  return stamp;
}

int main(int argc, char* argv[])
{
  // Initialize the ROS 2 context
  rclcpp::init(argc, argv);

  try {
    // Check the command-line arguments
    if ((argc != 3 && argc != 4) || (argc == 4 && std::string(argv[3]) != "-a")) {
      std::cerr << "Usage: " << argv[0] << " <input_bag> <output_bag> [-a](keep all)" << std::endl;
      return 1;
    }

    if (argc == 3){
      rewriteBagTimestamps(argv[1], argv[2]);
    }
    else if (argc == 4){
      rewriteBagTimestamps(argv[1], argv[2], true); // Keep all topics
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // Shutdown the ROS 2 context
  rclcpp::shutdown();

  return 0;
}