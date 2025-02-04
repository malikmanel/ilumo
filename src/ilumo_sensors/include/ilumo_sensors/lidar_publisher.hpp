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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr side_point_cloud_pub_;
    rclcpp::Publisher<ilumo_interfaces::msg::ImuBurst>::SharedPtr side_imu_burst_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr side_avg_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr side_temp_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bottom_point_cloud_pub_;
    rclcpp::Publisher<ilumo_interfaces::msg::ImuBurst>::SharedPtr bottom_imu_burst_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr bottom_avg_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr bottom_temp_pub_;

    sensor_msgs::msg::PointCloud2::SharedPtr side_point_cloud_msg;
    ilumo_interfaces::msg::ImuBurst side_imu_burst_msg;
    sensor_msgs::msg::Imu side_avg_imu_msg;
    sensor_msgs::msg::Temperature side_temp_msg;

    sensor_msgs::msg::PointCloud2::SharedPtr bottom_point_cloud_msg;
    ilumo_interfaces::msg::ImuBurst bottom_imu_burst_msg;
    sensor_msgs::msg::Imu bottom_avg_imu_msg;
    sensor_msgs::msg::Temperature bottom_temp_msg;
  
    rclcpp::TimerBase::SharedPtr side_point_cloud_timer_;
    rclcpp::TimerBase::SharedPtr side_imu_timer_;

    rclcpp::TimerBase::SharedPtr bottom_point_cloud_timer_;
    rclcpp::TimerBase::SharedPtr bottom_imu_timer_;

    rclcpp::CallbackGroup::SharedPtr side_point_cloud_callback_group;
    rclcpp::CallbackGroup::SharedPtr side_imu_callback_group;
    rclcpp::CallbackGroup::SharedPtr bottom_point_cloud_callback_group;
    rclcpp::CallbackGroup::SharedPtr bottom_imu_callback_group;

    void pointcloudCallback(std::shared_ptr<blickfeld::scanner::point_cloud_stream<blickfeld::protocol::data::Frame>> point_cloud_stream,
                            sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg,
                            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_);
    void imuCallback(std::shared_ptr<blickfeld::imu_stream> imu_stream,
                     ilumo_interfaces::msg::ImuBurst imu_burst_msg,
                     sensor_msgs::msg::Imu avg_imu_msg,
                     rclcpp::Publisher<ilumo_interfaces::msg::ImuBurst>::SharedPtr imu_burst_pub_,
                     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr avg_imu_pub_);

    std::shared_ptr<blickfeld::scanner> side_scanner;
    std::shared_ptr<blickfeld::scanner::point_cloud_stream<blickfeld::protocol::data::Frame> > side_point_cloud_stream;
    std::shared_ptr<blickfeld::imu_stream> side_imu_stream;

    std::shared_ptr<blickfeld::scanner> bottom_scanner;
    std::shared_ptr<blickfeld::scanner::point_cloud_stream<blickfeld::protocol::data::Frame> > bottom_point_cloud_stream;
    std::shared_ptr<blickfeld::imu_stream> bottom_imu_stream;

    uint64_t last_pc_side_ts;
    uint64_t last_pc_bottom_ts;
    uint64_t last_imu_side_ts;
    uint64_t last_imu_bottom_ts;
};

#endif // LIDAR_PUBLISHER_HPP