#include "sensor_publishers/stereo_camera_publisher.hpp"
#include "zed2_interface/videocapture.hpp"
#include "zed2_interface/sensorcapture.hpp"

#include <rclcpp/rclcpp.hpp>

StereoCameraPublisher::StereoCameraPublisher(const std::string& name) : Node(name)
{
  // publishers for all camera info. Still needs barometer and all that. Also prob. split IMU into multiple publishers
  left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/left_camera/image", 10);
  right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/right_camera/image", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("stereo_camera/imu", 10);
  magnetometer_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("stereo_camera/magnetometer", 10);
  pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("stereo_camera/environment/pressure", 10);
  temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/environment/temperature", 10);;
  humidity_pub_ = create_publisher<sensor_msgs::msg::RelativeHumidity>("stereo_camera/environment/humidity", 10);;
  camera_temperature_left_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/left_camera/temperature", 10);;
  camera_temperature_right_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/right_camera/temperature", 10);;

  // create some sort of subscriber here
  // could actually be a timerbased callback that samples the Zed2 data at 100Hz to see if there is new info
  // Probably better to write a function that creates a camera stream and have the callback activate every time a new image comes from the camera stream

  // Set the verbose level
  sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

  // Create a SensorCapture object
  sl_oc::sensors::SensorCapture sens(verbose);

  // ----> Get a list of available camera with sensor
  std::vector<int> devs = sens.getDeviceList();

  if( devs.size()==0 )
  {
      RCLCPP_ERROR(get_logger(), "No available ZED Mini or ZED2 cameras");
      rclcpp::shutdown();
  }
  // <---- Get a list of available camera with sensor

  // ----> Inizialize the sensors
  if( !sens.initializeSensors( devs[0] ) )
  {
      std::cerr << "Connection failed" << std::endl;
      RCLCPP_ERROR(get_logger(), "Connection to ZED2 failed");
  }

  RCLCPP_INFO(get_logger(), "Sensor Capture connected to camera sn: " + sens.getSerialNumber());
  // <---- Inizialize the sensors

  // ----> Get FW version information
  uint16_t fw_maior;
  uint16_t fw_minor;

  sens.getFirmwareVersion( fw_maior, fw_minor );

  RCLCPP_INFO(get_logger(), " * Firmware version: " << std::to_string(fw_maior).c_str() << "." << std::to_string(fw_minor).c_str());
  // <---- Get FW version information

  // ----> Variables to calculate sensors frequencies
  uint64_t last_imu_ts = 0;
  uint64_t last_mag_ts = 0;
  uint64_t last_env_ts = 0;
  uint64_t last_cam_temp_ts = 0;
  // <---- Variables to calculate sensors frequencies

  // if else statement here to ensure this only prints when true
  RCLCPP_INFO(get_logger(), "Stereo camera active and publishing.");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}