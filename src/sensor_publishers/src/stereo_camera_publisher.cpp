#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class StereoCameraPublisher : public rclcpp::Node
{
public:
  StereoCameraPublisher() : Node("stereo_camera_publisher")
  {
    // publishers for all camera info. Still needs barometer and all that. Also prob. split IMU into multiple publishers
    left_image_pub_ = create_publisher<std_msgs::msg::String>("stereo_camera/left_image", 10);
    right_image_pub_ = create_publisher<std_msgs::msg::String>("stereo_camera/right_image", 10);
    imu_pub_ = create_publisher<std_msgs::msg::String>("stereo_camera/imu", 10);

    // create some sort of subscriber here
    // could actually be a timerbased callback that samples the Zed2 data at 100Hz to see if there is new info
    // publishers if new info is available

    // if else statement here to ensure this only prints when true
    RCLCPP_INFO(get_logger(), "Stereo camera active and publishing.");
  }

  void stereocameraCallback()
  {

  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr left_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr right_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr imu_pub_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}