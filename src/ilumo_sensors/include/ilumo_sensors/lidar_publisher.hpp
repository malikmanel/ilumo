#ifndef LIDAR_PUBLISHER_HPP
#define LIDAR_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <blickfeld/scanner.h>

class LiDARPublisher : public rclcpp::Node
{
public:
    LiDARPublisher(const std::string &name);
    // This might need a special desctructor setting stream = nullptr;

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  
    rclcpp::TimerBase::SharedPtr point_cloud_timer_;

    void pointcloudCallback();

    std::shared_ptr<blickfeld::scanner> scanner;
    std::shared_ptr<blickfeld::scanner::point_cloud_stream<blickfeld::protocol::data::Frame> > stream;
};

#endif // LIDAR_PUBLISHER_HPP