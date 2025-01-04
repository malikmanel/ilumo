#ifndef LIDAR_PUBLISHER_HPP
#define LIDAR_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include "ilumo_interfaces/msg/imu_burst.hpp"

#include <blickfeld/scanner.h>

class LiDARPublisher : public rclcpp::Node
{
public:
    LiDARPublisher(const std::string &name);
    // This might need a special desctructor setting stream = nullptr;

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<ilumo_interfaces::msg::ImuBurst>::SharedPtr imu_burst_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr avg_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;

    sensor_msgs::msg::PointCloud2 *point_cloud_msg;
    ilumo_interfaces::msg::ImuBurst imu_burst_msg;
    sensor_msgs::msg::Imu avg_imu_msg;
    sensor_msgs::msg::Temperature temp_msg;
  
    rclcpp::TimerBase::SharedPtr point_cloud_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;

    void pointcloudCallback();
    void imuCallback();

    std::shared_ptr<blickfeld::scanner> scanner;
    std::shared_ptr<blickfeld::scanner::point_cloud_stream<blickfeld::protocol::data::Frame> > point_cloud_stream;
    std::shared_ptr<blickfeld::imu_stream> imu_stream;
};

#endif // LIDAR_PUBLISHER_HPP