#include "ilumo_description/ilumo_kinematics.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;


IlumoKinematics::IlumoKinematics(const std::string& name): Node(name)
{
    // Quaternion for calculations from radian
    tf2::Quaternion q;

    // ----> Setup broadcasters
    stereo_camera_top_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    stereo_camera_bottom_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    stereo_camera_center_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    lidar_side_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    lidar_bottom_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    thermal_camera_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // <---- Setup broadcasters

    // ----> Setup top stereo camera transform
    stereo_camera_top_tf_stamped_.header.stamp = get_clock()->now();
    stereo_camera_top_tf_stamped_.header.frame_id = "ilumo_base";
    stereo_camera_top_tf_stamped_.child_frame_id = "stereo_camera_top";
    stereo_camera_top_tf_stamped_.transform.translation.x = 0.065683;
    stereo_camera_top_tf_stamped_.transform.translation.y = -0.00067464;
    stereo_camera_top_tf_stamped_.transform.translation.z = 0.17;

    q.setRPY(1.5708, 0.0, 0.0);

    stereo_camera_top_tf_stamped_.transform.rotation.x = q.x();
    stereo_camera_top_tf_stamped_.transform.rotation.y = q.y();
    stereo_camera_top_tf_stamped_.transform.rotation.z = q.z();
    stereo_camera_top_tf_stamped_.transform.rotation.w = q.w();
    // ----> Setup top stereo camera transform

    // ----> Setup bottom stereo camera transform
    stereo_camera_bottom_tf_stamped_.header.stamp = get_clock()->now();
    stereo_camera_bottom_tf_stamped_.header.frame_id = "ilumo_base";
    stereo_camera_bottom_tf_stamped_.child_frame_id = "stereo_camera_bottom";
    stereo_camera_bottom_tf_stamped_.transform.translation.x = 0.065683;
    stereo_camera_bottom_tf_stamped_.transform.translation.y = -0.00067464;
    stereo_camera_bottom_tf_stamped_.transform.translation.z = 0.05;

    q.setRPY(1.5708, 0.0, 0.0);

    stereo_camera_bottom_tf_stamped_.transform.rotation.x = q.x();
    stereo_camera_bottom_tf_stamped_.transform.rotation.y = q.y();
    stereo_camera_bottom_tf_stamped_.transform.rotation.z = q.z();
    stereo_camera_bottom_tf_stamped_.transform.rotation.w = q.w();
    // <---- Setup bottom stereo camera transform

    // ----> Setup stereo camera center transform
    stereo_camera_center_tf_stamped_.header.stamp = get_clock()->now();
    stereo_camera_center_tf_stamped_.header.frame_id = "ilumo_base";
    stereo_camera_center_tf_stamped_.child_frame_id = "stereo_camera_center";
    stereo_camera_center_tf_stamped_.transform.translation.x = 0.0685672;
    stereo_camera_center_tf_stamped_.transform.translation.y = -0.00067464;
    stereo_camera_center_tf_stamped_.transform.translation.z = 0.11;

    q.setRPY(1.5708, 0.0, 0.0);

    stereo_camera_center_tf_stamped_.transform.rotation.x = q.x();
    stereo_camera_center_tf_stamped_.transform.rotation.y = q.y();
    stereo_camera_center_tf_stamped_.transform.rotation.z = q.z();
    stereo_camera_center_tf_stamped_.transform.rotation.w = q.w();
    // <---- Setup stereo camera center transform

    // ----> Setup lidar side transform
    lidar_side_tf_stamped_.header.stamp = get_clock()->now();
    lidar_side_tf_stamped_.header.frame_id = "ilumo_base";
    lidar_side_tf_stamped_.child_frame_id = "lidar_side";
    lidar_side_tf_stamped_.transform.translation.x = 0.0655;
    lidar_side_tf_stamped_.transform.translation.y = -0.00067464;
    lidar_side_tf_stamped_.transform.translation.z = -0.0472;
    
    q.setRPY(1.5708, 1.5708, 0.0);

    lidar_side_tf_stamped_.transform.rotation.x = q.x();
    lidar_side_tf_stamped_.transform.rotation.y = q.y();
    lidar_side_tf_stamped_.transform.rotation.z = q.z();
    lidar_side_tf_stamped_.transform.rotation.w = q.w();
    // <---- Setup lidar side camera transform

    // ----> Setup lidar bottom transform
    lidar_bottom_tf_stamped_.header.stamp = get_clock()->now();
    lidar_bottom_tf_stamped_.header.frame_id = "ilumo_base";
    lidar_bottom_tf_stamped_.child_frame_id = "lidar_bottom";
    lidar_bottom_tf_stamped_.transform.translation.x = -0.042874;
    lidar_bottom_tf_stamped_.transform.translation.y = 0.0017;
    lidar_bottom_tf_stamped_.transform.translation.z = -0.058344;

    q.setRPY(-1.5708, 0.5236, 0.0);

    lidar_bottom_tf_stamped_.transform.rotation.x = q.x();
    lidar_bottom_tf_stamped_.transform.rotation.y = q.y();
    lidar_bottom_tf_stamped_.transform.rotation.z = q.z();
    lidar_bottom_tf_stamped_.transform.rotation.w = q.w();
    // <---- Setup lidar bottom camera transform

    // ----> Setup thermal camera transform
    thermal_camera_tf_stamped_.header.stamp = get_clock()->now();
    thermal_camera_tf_stamped_.header.frame_id = "ilumo_base";
    thermal_camera_tf_stamped_.child_frame_id = "thermal_camera";
    thermal_camera_tf_stamped_.transform.translation.x = 0.096003;
    thermal_camera_tf_stamped_.transform.translation.y = 0.0;
    thermal_camera_tf_stamped_.transform.translation.z = 0.10277;

    q.setRPY(0.0, 0.0, 0.0);

    thermal_camera_tf_stamped_.transform.rotation.x = q.x();
    thermal_camera_tf_stamped_.transform.rotation.y = q.y();
    thermal_camera_tf_stamped_.transform.rotation.z = q.z();
    thermal_camera_tf_stamped_.transform.rotation.w = q.w();
    // <---- Setup thermal camera transform

    // ----> Broadcast transforms
    stereo_camera_top_tf_broadcaster_->sendTransform(stereo_camera_top_tf_stamped_);
    stereo_camera_bottom_tf_broadcaster_->sendTransform(stereo_camera_bottom_tf_stamped_);
    stereo_camera_center_tf_broadcaster_->sendTransform(stereo_camera_center_tf_stamped_);
    lidar_side_tf_broadcaster_->sendTransform(lidar_side_tf_stamped_);
    lidar_bottom_tf_broadcaster_->sendTransform(lidar_bottom_tf_stamped_);
    thermal_camera_tf_broadcaster_->sendTransform(thermal_camera_tf_stamped_);
    // <---- Broadcast transforms

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/ilumo_controller/motor_state", 10,
      std::bind(&IlumoKinematics::handleExternalConnectorPose, this, std::placeholders::_1));
    }


void IlumoKinematics::handleExternalConnectorPose(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{  
    // Quaternion for calculations from radian
    tf2::Quaternion q;

    // ----> Setup external connector transform
    external_connector_tf_stamped_.header.stamp = get_clock()->now();
    external_connector_tf_stamped_.header.frame_id = "ilumo_base";
    external_connector_tf_stamped_.child_frame_id = "external_connector";
    external_connector_tf_stamped_.transform.translation.x = 0;
    external_connector_tf_stamped_.transform.translation.y = 0;
    external_connector_tf_stamped_.transform.translation.z = 0.344;

    q.setRPY(0.0, 0.0, msg->position[0]);

    external_connector_tf_stamped_.transform.rotation.x = q.x();
    external_connector_tf_stamped_.transform.rotation.y = q.y();
    external_connector_tf_stamped_.transform.rotation.z = q.z();
    external_connector_tf_stamped_.transform.rotation.w = q.w();
    // <---- Setup external connector transform

    // Broadcast transform
    external_connector_tf_broadcaster_->sendTransform(external_connector_tf_stamped_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IlumoKinematics>("ilumo_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}