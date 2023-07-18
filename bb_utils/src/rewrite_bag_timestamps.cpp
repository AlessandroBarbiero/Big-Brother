// C++
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <chrono>
// ROS
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/serialization.hpp"
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

// To run this type:
// ros2 run bb_utils rewrite_bag_timestamps /path/to/input.bag /path/to/output.bag
// Example:
// ros2 run bb_utils rewrite_bag_timestamps /home/ale/bag_files/static_sensors_only /home/ale/bag_files/static_sensors_only_correct_ts

builtin_interfaces::msg::Time getStamp(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message, std::string topic_type);

// Function that rewrites the timestamps inside the input_bag to match the timestamps inside the messages and publishes a new bag
void rewriteBagTimestamps(const std::string& input_bag, const std::string& output_bag)
{

  std::vector<std::string> topics_to_keep = {
                                    "/carla/sensors_home/static_rgb_camera/image", 
                                    "/carla/sensors_home/static_depth_camera/image",
                                    "/carla/sensors_home/static_termic_camera/image",
                                    "/carla/sensors_home/static_rgb_camera/camera_info", 
                                    "/carla/sensors_home/static_depth_camera/camera_info",
                                    "/carla/sensors_home/static_termic_camera/camera_info",
                                    "/carla/sensors_home/static_lidar",
                                    "/tf",
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
  for (auto topic : topics_to_keep) {
      storage_filter.topics.push_back(topic);
  }
  reader->set_filter(storage_filter);

  // Recreate the topics in the new bag
  std::unordered_map<std::string, std::string> topic_to_type;
  const auto topics = reader->get_all_topics_and_types();
  for (const auto topic : topics){
    auto iter = std::find(topics_to_keep.begin(), topics_to_keep.end(), topic.name);
    if(iter != topics_to_keep.end()){
      topic_to_type.emplace(topic.name, topic.type);
      writer->create_topic(
        {topic.name,
        topic.type,
        rmw_get_serialization_format(),
        ""});
    }
  }

  // Iterate over messages in the input bag
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;
  std::string topic_type;
  builtin_interfaces::msg::Time stamp;
  std::cout<< "Start reading the bag..."<<std::endl;

  auto start_time = std::chrono::high_resolution_clock::now();

  while (rclcpp::ok() && reader->has_next())
  {
    // Read the next message
    bag_message = reader->read_next();
    topic_type = topic_to_type.at(bag_message->topic_name);

    // Get the timestamp from the header
    stamp = getStamp(bag_message, topic_type);
    const int64_t timestamp = int64_t(stamp.sec) * 1000000000 + int64_t(stamp.nanosec);

    // std::cout << "stamp: \t\t" << bag_message->time_stamp << std::endl;
    // std::cout << "header stamp: \t" << timestamp << std::endl;

    // Update the message timestamp
    bag_message->time_stamp=timestamp;

    // Write the modified message to the output bag
    writer->write(bag_message);
  }

  std::cout<< "Finished" << std::endl;
  // Stop the clock
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start_time;
  // Print the duration in seconds
  std::cout << "Execution time: " << duration.count() << " seconds." << std::endl;
}

builtin_interfaces::msg::Time getStamp(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message, std::string topic_type){

  static rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
  static rclcpp::Serialization<sensor_msgs::msg::CameraInfo> cam_info_serializer;
  static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> lidar_serializer;
  static rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer;
  static rclcpp::Serialization<rosgraph_msgs::msg::Clock> clock_serializer;

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

  return stamp;
}

int main(int argc, char* argv[])
{
  // Initialize the ROS 2 context
  rclcpp::init(argc, argv);

  try {
    // Check the command-line arguments
    if (argc != 3) {
      std::cerr << "Usage: " << argv[0] << " <input_bag> <output_bag>" << std::endl;
      return 1;
    }

    // Rewrite the bag timestamps
    rewriteBagTimestamps(argv[1], argv[2]);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // Shutdown the ROS 2 context
  rclcpp::shutdown();

  return 0;
}