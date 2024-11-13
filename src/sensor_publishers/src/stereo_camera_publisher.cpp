#include "sensor_publishers/stereo_camera_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

StereoCameraPublisher::StereoCameraPublisher(const std::string& name) : Node(name)
{
  // publishers for all camera info. Still needs barometer and all that. Also prob. split IMU into multiple publishers
  left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/left_image", 10);
  right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/right_image", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("stereo_camera/imu", 10);
  magnetometer_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("stereo_camera/magnetometer", 10);

  // create some sort of subscriber here
  // could actually be a timerbased callback that samples the Zed2 data at 100Hz to see if there is new info
  // Probably better to write a function that creates a camera stream and have the callback activate every time a new image comes from the camera stream

  // if else statement here to ensure this only prints when true
  RCLCPP_INFO(get_logger(), "Stereo camera active and publishing.");
}

void StereoCameraPublisher::imageCallback()
{

}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}