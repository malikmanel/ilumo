#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher")
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
  }

  void timerCallback()
  {
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}