// ----> Includes
#include <chrono>
#include "ilumo_interfaces/msg/imu_single.hpp"

#include "ilumo_sensors/lidar_publisher.hpp"
// <---- Includes

using namespace std::chrono_literals;

template <typename FieldT>
void addPointCloudField(sensor_msgs::msg::PointCloud2& point_cloud_msg,
                        const std::string& name, 
                        uint32_t offset,
                        uint8_t datatype) 
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.count = 1;
  field.datatype = datatype;
  field.offset = offset;
  point_cloud_msg.fields.push_back(field);
  point_cloud_msg.point_step += sizeof(FieldT);
}

LiDARPublisher::LiDARPublisher(const std::string& name) : Node(name)
{
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_camera/point_cloud", 10);
    imu_burst_pub_ = this->create_publisher<ilumo_interfaces::msg::ImuBurst>("lidar_camera/imu_burst", 10);
    avg_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("lidar_camera/avg_imu", 10);
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("lidar_camera/temperature", 10);

    std::string scanner_ip_or_host = "localhost";

    // ----> Creating a connection to the device.
    std::shared_ptr<blickfeld::scanner> scanner = blickfeld::scanner::connect(scanner_ip_or_host);
    RCLCPP_INFO_STREAM(get_logger(),"Connected to the Blickfeld sensor at address " << scanner_ip_or_host);
    // <---- Creating a connection to the device.

    // Create a pointcloud stream object to receive pointclouds
	auto point_cloud_stream = scanner->get_point_cloud_stream();
    auto imu_stream = scanner->get_imu_stream();

    // ----> Prepare LiDAR messages
    auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    auto imu_burst_msg = ilumo_interfaces::msg::ImuBurst();
    auto avg_imu_msg = sensor_msgs::msg::Imu();
    auto temp_msg = sensor_msgs::msg::Temperature();

    // point_cloud_msg->header.frame_id = 
    // imu_burst_msg.frame_id = 
    // avg_imu_msg.header.frame_id = 

    addPointCloudField<float>(std::ref(*point_cloud_msg), "x", point_cloud_msg->point_step,
                              sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<float>(std::ref(*point_cloud_msg), "y", point_cloud_msg->point_step,
                              sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<float>(std::ref(*point_cloud_msg), "z", point_cloud_msg->point_step,
                              sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<uint32_t>(std::ref(*point_cloud_msg), "intensity", point_cloud_msg->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
    addPointCloudField<uint32_t>(std::ref(*point_cloud_msg), "ambient_light", point_cloud_msg->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
    addPointCloudField<float>(std::ref(*point_cloud_msg), "time_offset", point_cloud_msg->point_step,
                                 sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<uint32_t>(std::ref(*point_cloud_msg), "return_id", point_cloud_msg->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
    addPointCloudField<uint32_t>(std::ref(*point_cloud_msg), "point_id", point_cloud_msg->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
    // <---- Prepare LiDAR messages

    // ----> Initialize publishers
    point_cloud_timer_ = create_wall_timer(15ms, std::bind(&LiDARPublisher::pointcloudCallback, this));
    imu_timer_ = create_wall_timer(0.8ms, std::bind(&LiDARPublisher::imuCallback, this));
    // <---- Initialize publishers
}

void LiDARPublisher::pointcloudCallback()
{
    const blickfeld::protocol::data::Frame frame = point_cloud_stream->recv_frame();

    const auto number_of_points = frame.total_number_of_returns(); // change to total_number_of_points if we ignore returns

    /// reserve memory
    point_cloud_msg->data.resize(number_of_points * point_cloud_msg->point_step);

    /// set point cloud message data
    // point_cloud_msg->header.frame_id = frame_id
    point_cloud_msg->header.stamp.nanosec = frame.start_time_ns();
    point_cloud_msg->is_dense = false;
    point_cloud_msg->height = 1;
    point_cloud_msg->width = number_of_points;
    point_cloud_msg->row_step = point_cloud_msg->point_step * point_cloud_msg->width;

    int point_index = 0;

	// Iterate through all the scanlines in a frame
    for (int s_ind = 0; s_ind < frame.scanlines_size(); s_ind++) {

        // Iterate through all the points in a scanline
        for (int p_ind = 0; p_ind < frame.scanlines(s_ind).points_size(); p_ind++) {
            auto& point = frame.scanlines(s_ind).points(p_ind);
            auto time_offset = frame.scanlines(s_ind).start_offset_ns() + point.start_offset_ns();

            // Iterate through all the returns for each points
            // this might not be necessary, maybe the first return is enough
            for (int r_ind = 0; r_ind < point.returns_size(); r_ind++) {
                auto& ret = point.returns(r_ind);
                printf("coordinates: (%f, %f, %f)\n", ret.cartesian(0), ret.cartesian(1), ret.cartesian(2));

                // also relevant: ret.intensity()
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[0].offset] = ret.cartesian(0);
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[1].offset] = ret.cartesian(0);
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[2].offset] = ret.cartesian(0);
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[3].offset] = ret.intensity();
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[4].offset] = point.ambient_light_level();
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[5].offset] = time_offset;
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[6].offset] = ret.id();
                point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[7].offset] = point.id();

                point_index++;
            }
        }
    }

    point_cloud_pub_->publish(*point_cloud_msg);
    temp_pub_->publish(temp_msg);
}

void LiDARPublisher::imuCallback()
{
    const blickfeld::protocol::data::IMU data = imu_stream->recv_burst();
    // received as bursts of data, so a lower frequency is possible. how do I know how large a burst ist?

    const auto number_of_samples = data.packed().length();

    auto burst_timestamp = data.start_time_ns();
    float total_ax = 0.0;
    float total_ay = 0.0;
    float total_az = 0.0;
    float total_rx = 0.0;
    float total_ry = 0.0;
    float total_rz = 0.0;

    for (auto sample : data.samples()) {
        // Creating a single IMU msg
        auto single_imu_msg = ilumo_interfaces::msg::ImuSingle();

        // Filling IMU msg with data
        single_imu_msg.stamp.nanosec = burst_timestamp + sample.start_offset_ns();
        single_imu_msg.angular_velocity.x = sample.angular_velocity(0);
        single_imu_msg.angular_velocity.y = sample.angular_velocity(1);
        single_imu_msg.angular_velocity.z = sample.angular_velocity(2);
        single_imu_msg.linear_acceleration.x = sample.acceleration(0);
        single_imu_msg.linear_acceleration.y = sample.acceleration(1);
        single_imu_msg.linear_acceleration.z = sample.acceleration(2);

        // Adding IMU msg to burst msg
        imu_burst_msg.burst.push_back(single_imu_msg);

        // Integrating IMU msgs
        total_rx += sample.angular_velocity(0);
        total_ry += sample.angular_velocity(1);
        total_rz += sample.angular_velocity(2);
        total_ax += sample.acceleration(0);
        total_ay += sample.acceleration(1);
        total_az += sample.acceleration(2);
    }

    // This assumes measurements at a constant frequency
    // Otherwise next_timestamp - current time_stamp, but hard to get
    // Maybe we could do current_timestamp - last_timestamp
    // Direction of time should theoretically not matter

    // Filling mean IMU message with data
    avg_imu_msg.header.stamp.nanosec = burst_timestamp;
    avg_imu_msg.angular_velocity.x = total_rx / number_of_samples;
    avg_imu_msg.angular_velocity.y = total_ry / number_of_samples;
    avg_imu_msg.angular_velocity.z = total_rz / number_of_samples;
    avg_imu_msg.linear_acceleration.x = total_ax / number_of_samples;
    avg_imu_msg.linear_acceleration.y = total_ay / number_of_samples;
    avg_imu_msg.linear_acceleration.z = total_az / number_of_samples;

    // Publishing messages
    imu_burst_pub_->publish(imu_burst_msg);
    avg_imu_pub_->publish(avg_imu_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARPublisher>("lidar_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}