#include <iostream>
#include <string>
#include <rosbag2_cpp/converter.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/serialization_format_converter_factory.hpp>
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>
#include <rclcpp/rclcpp.hpp>

// To run this type:
// ros2 run bb_utils rewrite_bag_timestamps /path/to/input.bag /path/to/output.bag


// Custom function to rewrite bag timestamps
void rewriteBagTimestamps(const std::string& input_bag, const std::string& output_bag)
{
  std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader;
  rosbag2_storage::StorageFilter storage_filter;
  //storage_filter.topics.push_back("/carla/sensors_home/static_depth_camera/image");
  storage_filter.topics.push_back("/carla/sensors_home/static_rgb_camera/image");
  //storage_filter.topics.push_back("/carla/sensors_home/static_termic_camera/image");
  //storage_filter.topics.push_back("/carla/sensors_home/static_lidar");
  
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer;

  // Configure the storage options for the reader and writer
  const rosbag2_cpp::StorageOptions r_storage_options({input_bag, "sqlite3"});
  const rosbag2_cpp::ConverterOptions converter_options(
    {rmw_get_serialization_format(),
      rmw_get_serialization_format()});

  // Open the input bag for reading
  reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  reader->open(r_storage_options, converter_options);
  reader->set_filter(storage_filter);

  const rosbag2_cpp::StorageOptions w_storage_options({output_bag, "sqlite3"});
  // Open the output bag for writing
  writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  writer->open(w_storage_options, converter_options);

// TODO: This is not working -> I need to read inside the header 
//      of the message and set the timestamp accordingly

  // const auto topics = reader->get_all_topics_and_types();

  // for (const auto topic : topics)
  //   printf(topic.name.c_str());


  // Iterate over messages in the input bag
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
  while (reader->has_next())
  {
    // Read the next message
    message = reader->read_next();

    // Deserialize the message
    // --- deserialized_message =

    // Get the timestamp from the header
    // const auto header_field = deserialized_message->get_header();
    // const auto stamp_field = header_field.find("stamp");
    // const auto timestamp = stamp_field->second;
    const auto timestamp = 0;

    // Update the message timestamp
    message->time_stamp=timestamp;

    // Write the modified message to the output bag
    writer->write(message);
  }
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