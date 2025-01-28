#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>


class IlumoKinematics : public rclcpp::Node
{
public:
    IlumoKinematics(const std::string& name);

private: 
    void handleExternalConnectorPose(const std::shared_ptr<sensor_msgs::msg::JointState> msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> stereo_camera_top_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> stereo_camera_bottom_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> stereo_camera_center_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_side_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_bottom_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> thermal_camera_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> external_connector_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> external_connector_tf_broadcaster_dynamic_;

    geometry_msgs::msg::TransformStamped stereo_camera_top_tf_stamped_;
    geometry_msgs::msg::TransformStamped stereo_camera_bottom_tf_stamped_;
    geometry_msgs::msg::TransformStamped stereo_camera_center_tf_stamped_;
    geometry_msgs::msg::TransformStamped lidar_side_tf_stamped_;
    geometry_msgs::msg::TransformStamped lidar_bottom_tf_stamped_;
    geometry_msgs::msg::TransformStamped thermal_camera_tf_stamped_;
    geometry_msgs::msg::TransformStamped external_connector_tf_stamped_;
};

#endif // SIMPLE_TF_KINEMATICS_HPP