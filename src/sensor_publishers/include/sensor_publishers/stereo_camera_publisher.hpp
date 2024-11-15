#ifndef STEREO_CAMERA_PUBLISHER_HPP
#define STEREO_CAMERA_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>

class StereoCameraPublisher : public rclcpp::Node
{
public:
    StereoCameraPublisher(const std::string &name);


private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr camera_temperature_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr camera_temperature_right_pub_;
};

#endif // STEREO_CAMERA_PUBLISHER_HPP