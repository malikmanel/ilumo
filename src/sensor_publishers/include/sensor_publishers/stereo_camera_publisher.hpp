#ifndef STEREO_CAMERA_PUBLISHER_HPP
#define STEREO_CAMERA_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

class StereoCameraPublisher : public rclcpp::Node
{
public:
    StereoCameraPublisher(const std::string &name);


private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_pub_;
    // Environment data: Pressure, Temperature, relative humidity
    // Camera temperature data: left camera, right camera
};

#endif // STEREO_CAMERA_PUBLISHER_HPP