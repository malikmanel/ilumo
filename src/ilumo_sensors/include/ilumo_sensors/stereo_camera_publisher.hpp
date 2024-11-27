#ifndef STEREO_CAMERA_PUBLISHER_HPP
#define STEREO_CAMERA_PUBLISHER_HPP

// ----> Includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>

#include "zed2_interface/videocapture.hpp"
#include "zed2_interface/sensorcapture.hpp"
// <---- Includes

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
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr left_camera_temperature_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr right_camera_temperature_pub_;

    sensor_msgs::msg::Image left_image_msg;
    sensor_msgs::msg::Image right_image_msg;
    sensor_msgs::msg::Imu imu_msg;
    sensor_msgs::msg::MagneticField mag_msg;
    sensor_msgs::msg::FluidPressure press_msg;
    sensor_msgs::msg::Temperature temp_msg;
    sensor_msgs::msg::RelativeHumidity humi_msg;
    sensor_msgs::msg::Temperature left_cam_temp_msg;
    sensor_msgs::msg::Temperature right_cam_temp_msg;

    rclcpp::TimerBase::SharedPtr image_timer_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;
    sl_oc::video::VideoCapture videoCap;
    sl_oc::sensors::SensorCapture sensCap;

    void imageCallback();
    void sensorCallback();

    uint64_t last_img_ts;
    uint64_t last_imu_ts;
    uint64_t last_mag_ts;
    uint64_t last_env_ts;
    uint64_t last_cam_temp_ts;
    float frame_fps;
    int width,height;
};

#endif // STEREO_CAMERA_PUBLISHER_HPP