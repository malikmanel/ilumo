// ----> Includes
#include <chrono>
#include <math.h>
#include "ilumo_interfaces/msg/imu_single.hpp"

#include "ilumo_sensors/lidar_publisher.hpp"
// <---- Includes

using namespace std::chrono_literals;
using namespace std::placeholders;

template <typename FieldT>
void addPointCloudField(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg,
                        const std::string& name, 
                        uint8_t datatype) 
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.count = 1;
    field.datatype = datatype;
    field.offset = point_cloud_msg->point_step;
    point_cloud_msg->fields.push_back(field);
    point_cloud_msg->point_step += sizeof(FieldT);
}

static inline int getPointCloud2FieldIndex(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg, 
                                           const std::string& field_name) {
    for (size_t field_index = 0; field_index < point_cloud_msg->fields.size(); ++field_index) {
      if (point_cloud_msg->fields[field_index].name == field_name) {
        return static_cast<int>(field_index);
      }
    }
    return -1;
  };

template <typename ValueT>
void assignField(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg, 
                    size_t point_index, 
                    std::string field_name, 
                    const ValueT& value) {
    uint32_t field_index = getPointCloud2FieldIndex(point_cloud_msg, field_name);
    *reinterpret_cast<ValueT*>(
        &point_cloud_msg->data[point_index * point_cloud_msg->point_step + point_cloud_msg->fields[field_index].offset]) =
        static_cast<ValueT>(value);
}

void makePointCloudMessage(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
{
    point_cloud_msg->point_step = 0;
    addPointCloudField<float>(point_cloud_msg, 
                              "x",
                              sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<float>(point_cloud_msg, 
                              "y",
                              sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<float>(point_cloud_msg, 
                              "z",
                              sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<uint32_t>(point_cloud_msg, 
                                 "intensity",
                                 sensor_msgs::msg::PointField::UINT32);
    addPointCloudField<uint32_t>(point_cloud_msg, 
                                 "ambient_light",
                                 sensor_msgs::msg::PointField::UINT32);
    addPointCloudField<float>(point_cloud_msg, 
                              "time_offset",
                              sensor_msgs::msg::PointField::FLOAT32);
    addPointCloudField<uint32_t>(point_cloud_msg, 
                                 "point_id", 
                                 sensor_msgs::msg::PointField::UINT32);
    addPointCloudField<uint32_t>(point_cloud_msg, 
                                 "return_id",
                                 sensor_msgs::msg::PointField::UINT32);
}

LiDARPublisher::LiDARPublisher(const std::string& name) : Node(name)
{
    // ----> Declare the parameters
    // LiDAR horizontal FoV
    auto hori_fov_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    hori_fov_param_desc.name = "Horizontal FoV";
    hori_fov_param_desc.type = 3; // Double
    hori_fov_param_desc.description = "Horizontal field of view in deg. (default=70)";
    hori_fov_param_desc.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                                .set__from_value(10.0)
                                                .set__to_value(70.0)};

    // LiDAR vertical FoV
    auto vert_fov_desc = rcl_interfaces::msg::ParameterDescriptor{};
    vert_fov_desc.name = "Vertical FoV";
    vert_fov_desc.type = 3; // Double
    vert_fov_desc.description = "Vertical field of view in deg. (default=20)";
    vert_fov_desc.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                          .set__from_value(5.0)
                                          .set__to_value(30.0)};

    // LiDAR horizontal pulse angle spacing
    auto pulse_space_desc = rcl_interfaces::msg::ParameterDescriptor{};
    pulse_space_desc.name = "Pulse Angle Spacing";
    pulse_space_desc.type = 3; // Double
    pulse_space_desc.description = "Horizontal pulse angle spacing in deg. Defines horizontal sample resolution. (default=0.4)";
    pulse_space_desc.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                             .set__from_value(0.4)
                                             .set__to_value(1.0)};

    // LiDAR vertical scan lines
    // Frame rate is supposedly equal to 2 * (natural mirror frequency)/(scan lines). 
    // From their example that suggests a natural frequency of 250Hz.
    // This would make 400 vertical scan  lines impossible.
    // Solution: Set scan lines, let blickfeld figure out the Hz.
    auto scan_lines_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    scan_lines_param_desc.name = "Scan lines";
    scan_lines_param_desc.type = 2; // Integer
    scan_lines_param_desc.description = "Vertical scan lines. More scan lines lead to a reduced frame rate. (default=350)";
    scan_lines_param_desc.integer_range = {rcl_interfaces::msg::IntegerRange()
                                           .set__from_value(5)
                                           .set__to_value(400)};

    // LiDAR node logging
    auto logging_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    logging_param_desc.name = "Verbosity";
    logging_param_desc.type = 2; // Integer
    logging_param_desc.description = "Logging verbosity of the LiDAR node. 0 = None, 1 = Error, 2 = Warning, 3 = Info (default=2)";
    logging_param_desc.integer_range = {rcl_interfaces::msg::IntegerRange()
                                        .set__from_value(0)
                                        .set__to_value(3)
                                        .set__step(1)};

    this->declare_parameter("horizontal_fov", 70.0, hori_fov_param_desc);
    this->declare_parameter("vertical_fov", 20.0, vert_fov_desc);
    this->declare_parameter("pulse_angle_space", 0.4, pulse_space_desc);
    this->declare_parameter("scan_lines", 350, scan_lines_param_desc);
    this->declare_parameter("verbosity", 2, logging_param_desc);
    // <---- Declare the parameters

    RCLCPP_INFO_STREAM(get_logger(), "Starting LiDAR camera node ...");

    // ----> Create the publishers
    side_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_camera/side_camera/point_cloud", 10);
    side_imu_burst_pub_ = this->create_publisher<ilumo_interfaces::msg::ImuBurst>("lidar_camera/side_camera/imu_burst", 10);
    side_avg_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("lidar_camera/side_camera/avg_imu", 10);

    bottom_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_camera/bottom_camera/point_cloud", 10);
    bottom_imu_burst_pub_ = this->create_publisher<ilumo_interfaces::msg::ImuBurst>("lidar_camera/bottom_camera/imu_burst", 10);
    bottom_avg_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("lidar_camera/bottom_camera/avg_imu", 10);
    // <---- Create the publishers

    // ----> Creating a connection to the device.
    std::string side_scanner_ip = "192.168.26.2";
    // std::string bottom_scanner_ip = "192.168.26.3";
    side_scanner = blickfeld::scanner::connect(side_scanner_ip);
    // bottom_scanner = blickfeld::scanner::connect(bottom_scanner_ip);
    if( false )
    {
        RCLCPP_ERROR(get_logger(), "Cannot open connection to the LiDAR (side).");
    }
     if( false )
    {
        RCLCPP_ERROR(get_logger(), "Cannot open connection to the LiDAR (bottom).");
    }

    RCLCPP_INFO_STREAM(get_logger(),"Connected to the LiDAR sensor (side) at address " << side_scanner_ip);
    // RCLCPP_INFO_STREAM(get_logger(),"Connected to the LiDAR sensor (bottom) at address " << bottom_scanner_ip);
    // <---- Creating a connection to the device.

    // ----> Set the scanner parameters
    blickfeld::protocol::config::ScanPattern scan_pattern;

    double hori_fov = this->get_parameter("horizontal_fov").as_double();
    double vert_fov = this->get_parameter("vertical_fov").as_double();
    double angle_space = this->get_parameter("pulse_angle_space").as_double();
    int scan_lines = this->get_parameter("scan_lines").as_int();

    int scan_lines_down = scan_lines / 2;
    int scan_lines_up = scan_lines - scan_lines_down;

    scan_pattern.mutable_horizontal()->set_fov(hori_fov * (M_PI / 180.0));
	scan_pattern.mutable_vertical()->set_fov(vert_fov * (M_PI / 180.0));
	scan_pattern.mutable_vertical()->set_scanlines_up(scan_lines_up); // Upramping phase (increase of vertical mirror movement to reach outer scanlines)
	scan_pattern.mutable_vertical()->set_scanlines_down(scan_lines_down); // Upramping phase (decrease of vertical mirror movement to return to inner scanlines)
    scan_pattern.mutable_pulse()->set_angle_spacing(angle_space * (M_PI / 180.0));

	scan_pattern = side_scanner->fill_scan_pattern(scan_pattern);
	side_scanner->set_scan_pattern(scan_pattern);
    //bottom_scanner->set_scan_pattern(scan_pattern);
    // <---- Set the scanner parameters

    // ----> Create a pointcloud stream object to receive pointclouds
	side_point_cloud_stream = side_scanner->get_point_cloud_stream();
    side_imu_stream = side_scanner->get_imu_stream();
    //bottom_point_cloud_stream = side_scanner->get_point_cloud_stream();
    //bottom_imu_stream = bottom_scanner->get_imu_stream();
    // <---- Create a pointcloud stream object to receive pointclouds

    // ----> Prepare LiDAR messages
    side_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    makePointCloudMessage(side_point_cloud_msg);
    side_imu_burst_msg = ilumo_interfaces::msg::ImuBurst();
    side_avg_imu_msg = sensor_msgs::msg::Imu();

    bottom_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    makePointCloudMessage(bottom_point_cloud_msg);
    bottom_imu_burst_msg = ilumo_interfaces::msg::ImuBurst();
    bottom_avg_imu_msg = sensor_msgs::msg::Imu();

    side_point_cloud_msg->header.frame_id = "lidar_side";
    side_imu_burst_msg.frame_id = "lidar_side";
    side_avg_imu_msg.header.frame_id = "lidar_side";

    bottom_point_cloud_msg->header.frame_id = "lidar_bottom";
    bottom_imu_burst_msg.frame_id = "lidar_bottom";
    bottom_avg_imu_msg.header.frame_id = "lidar_bottom";
    // <---- Prepare LiDAR messages

    // ----> Set previous timestamps to calculate frequency (debugging)
    last_pc_side_ts = 0;
    last_pc_bottom_ts = 0;
    last_imu_side_ts = 0;
    last_imu_bottom_ts = 0;
    // ----> Set previous timestamps to calculate frequency

    // ----> Initialize publishers
    side_point_cloud_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    side_imu_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    bottom_point_cloud_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    bottom_imu_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    side_point_cloud_timer_ = create_wall_timer(20ms, [this,
                                                       &point_cloud_stream = this->side_point_cloud_stream, 
                                                       &point_cloud_msg = this->side_point_cloud_msg,
                                                       &point_cloud_pub_ = this->side_point_cloud_pub_]()
                                                       -> void { LiDARPublisher::pointcloudCallback(
                                                        point_cloud_stream,
                                                        point_cloud_msg,
                                                        point_cloud_pub_
                                                        ); },
                                                side_point_cloud_callback_group
                                               );
    side_imu_timer_ = create_wall_timer(40ms, [this,
                                                &imu_stream = this->side_imu_stream,
                                                &imu_burst_msg = this->side_imu_burst_msg,
                                                &avg_imu_msg = this->side_avg_imu_msg,
                                                &imu_burst_pub_ = this->side_imu_burst_pub_,
                                                &avg_imu_pub_ = this->side_avg_imu_pub_]()
                                                -> void { LiDARPublisher::imuCallback(
                                                    imu_stream,
                                                    imu_burst_msg,
                                                    avg_imu_msg,
                                                    imu_burst_pub_,
                                                    avg_imu_pub_
                                                ); },
                                        side_imu_callback_group
                                       );
                                      
    /*
    bottom_point_cloud_timer_ = create_wall_timer(20ms, [this,
                                                         bottom_point_cloud_stream = this->bottom_point_cloud_stream, 
                                                         bottom_point_cloud_msg = this->bottom_point_cloud_msg,
                                                         bottom_point_cloud_pub_ = this->bottom_point_cloud_pub_]()
                                                         -> void { LiDARPublisher::pointcloudCallback(
                                                            bottom_point_cloud_stream,
                                                            bottom_point_cloud_msg,
                                                            bottom_point_cloud_pub_
                                                         ); },
                                                  bottom_point_cloud_callback_group
                                                 );
    bottom_imu_timer_ = create_wall_timer(40ms, [this,
                                                  bottom_imu_stream = this->bottom_imu_stream,
                                                  bottom_imu_burst_msg = this->bottom_imu_burst_msg,
                                                  bottom_avg_imu_msg = this->bottom_avg_imu_msg,
                                                  bottom_imu_burst_pub_ = this->bottom_imu_burst_pub_,
                                                  bottom_avg_imu_pub_ = this->bottom_avg_imu_pub_]()
                                                  -> void { LiDARPublisher::imuCallback(
                                                    bottom_imu_stream,
                                                    bottom_imu_burst_msg,
                                                    bottom_avg_imu_msg,
                                                    bottom_imu_burst_pub_,
                                                    bottom_avg_imu_pub_
                                                  ); },
                                          bottom_imu_callback_group
                                         );*/
    // <---- Initialize publishers
}

void LiDARPublisher::pointcloudCallback(std::shared_ptr<blickfeld::scanner::point_cloud_stream<blickfeld::protocol::data::Frame>> point_cloud_stream,
                                        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg,
                                        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_)
{
    // ----> Get LiDAR pointcloud frame 
    const blickfeld::protocol::data::Frame frame = point_cloud_stream->recv_frame();
    // <---- Get LiDAR pointcloud frame 

    // ----> LiDAR pointcloud debug information
    RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Point cloud timestamp (" 
                                      << point_cloud_msg->header.frame_id << "): " 
                                      << static_cast<double>(frame.start_time_ns())/1e9<< " sec" );
    if (point_cloud_msg->header.frame_id == "lidar_side"){
        if(last_pc_side_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(frame.start_time_ns()-last_pc_side_ts) << " Hz]");
        last_pc_side_ts = frame.start_time_ns();
    } else if (point_cloud_msg->header.frame_id == "lidar_bottom") {
        if(last_pc_bottom_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(frame.start_time_ns()-last_pc_bottom_ts) << " Hz]");
        last_pc_bottom_ts = frame.start_time_ns();
    }
    // <---- LiDAR pointcloud debug information

    // ----> Prepare point cloud messages
    // Calculate and reserve required memory
    const auto number_of_points = frame.total_number_of_returns(); // change to total_number_of_points if we ignore returns
    point_cloud_msg->data.resize(number_of_points * point_cloud_msg->point_step);

    // set point cloud message data
    point_cloud_msg->header.stamp.sec = frame.start_time_ns() / 1000000000;
    point_cloud_msg->header.stamp.nanosec = frame.start_time_ns() % 1000000000;
    point_cloud_msg->is_dense = false;
    point_cloud_msg->height = 1;
    point_cloud_msg->width = number_of_points;
    point_cloud_msg->row_step = point_cloud_msg->point_step * point_cloud_msg->width;
    // <---- Prepare point cloud messages

    // ----> Fill point cloud message
    int point_index = 0;
	// Iterate through all the scanlines in a frame
    for (int s_ind = 0; s_ind < frame.scanlines_size(); s_ind++) {

        // Iterate through all the points in a scanline
        for (int p_ind = 0; p_ind < frame.scanlines(s_ind).points_size(); p_ind++) {
            auto& point = frame.scanlines(s_ind).points(p_ind);
            auto time_offset = frame.scanlines(s_ind).start_offset_ns() + point.start_offset_ns();

            // Iterate through all the returns for each points
            for (int r_ind = 0; r_ind < point.returns_size(); r_ind++) {
                auto& ret = point.returns(r_ind);

                // Set coordinates, intensity, ambient ligh, time and id for each point
                assignField(point_cloud_msg, point_index, "x", ret.cartesian(0));
                assignField(point_cloud_msg, point_index, "y", ret.cartesian(1));
                assignField(point_cloud_msg, point_index, "z", ret.cartesian(2));
                assignField(point_cloud_msg, point_index, "intensity", ret.intensity());
                assignField(point_cloud_msg, point_index, "ambient_light", point.ambient_light_level());
                assignField(point_cloud_msg, point_index, "time_offset", time_offset);
                assignField(point_cloud_msg, point_index, "point_id", point.id());
                assignField(point_cloud_msg, point_index, "return_id", ret.id());

                point_index++;
            }
        }
    }
    // <---- Fill point cloud message

    // ----> Publish point cloud message
    point_cloud_pub_->publish(*point_cloud_msg);
    // <---- Publish point cloud message
}

void LiDARPublisher::imuCallback(std::shared_ptr<blickfeld::imu_stream> imu_stream,
                                 ilumo_interfaces::msg::ImuBurst imu_burst_msg,
                                 sensor_msgs::msg::Imu avg_imu_msg,
                                 rclcpp::Publisher<ilumo_interfaces::msg::ImuBurst>::SharedPtr imu_burst_pub_,
                                 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr avg_imu_pub_)
{
    // ----> Get LiDAR IMU data 
    const blickfeld::protocol::data::IMU data = imu_stream->recv_burst();
    // <---- Get LiDAR IMU data 

    // ----> LiDAR IMU debug information
    RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(9) << "IMU data timestamp (" 
                                    << avg_imu_msg.header.frame_id << "): " 
                                    << static_cast<double>(data.start_time_ns())/1e9<< " sec" );
    if (avg_imu_msg.header.frame_id == "lidar_side"){
        if(last_imu_side_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(data.start_time_ns()-last_imu_side_ts)*50 << " Hz]");
        last_pc_side_ts = data.start_time_ns();
    } else if (avg_imu_msg.header.frame_id == "lidar_bottom") {
        if(last_imu_bottom_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(data.start_time_ns()-last_imu_bottom_ts)*50 << " Hz]");
        last_pc_bottom_ts = data.start_time_ns();
    }
    // <---- LiDAR IMU debug information

    // ----> Setting message variables
    float number_of_samples = 0.0;

    auto burst_timestamp = data.start_time_ns();
    float total_ax = 0.0;
    float total_ay = 0.0;
    float total_az = 0.0;
    float total_rx = 0.0;
    float total_ry = 0.0;
    float total_rz = 0.0;
    // <---- Setting message variables

    // ----> Preparing burst IMU message
    for (auto sample : data.samples()) {
        // Creating a single IMU msg
        auto single_imu_msg = ilumo_interfaces::msg::ImuSingle();

        // Filling IMU msg with data
        auto imu_timestamp = burst_timestamp + sample.start_offset_ns();
        single_imu_msg.stamp.sec = imu_timestamp / 1000000000;
        single_imu_msg.stamp.nanosec = imu_timestamp % 1000000000;
        single_imu_msg.stamp.nanosec = burst_timestamp + sample.start_offset_ns();
        single_imu_msg.angular_velocity.x = sample.angular_velocity(0);
        single_imu_msg.angular_velocity.y = sample.angular_velocity(1);
        single_imu_msg.angular_velocity.z = sample.angular_velocity(2);
        single_imu_msg.linear_acceleration.x = sample.acceleration(0) * 9.80665;
        single_imu_msg.linear_acceleration.y = sample.acceleration(1) * 9.80665;
        single_imu_msg.linear_acceleration.z = sample.acceleration(2) * 9.80665;

        // Adding IMU msg to burst msg
        imu_burst_msg.burst.push_back(single_imu_msg);

        // Integrating IMU msgs
        total_rx += sample.angular_velocity(0);
        total_ry += sample.angular_velocity(1);
        total_rz += sample.angular_velocity(2);
        total_ax += sample.acceleration(0) * 9.80665;
        total_ay += sample.acceleration(1) * 9.80665;
        total_az += sample.acceleration(2) * 9.80665;

        // Tracking message amount
        number_of_samples++;
    }
    // <---- Preparing burst IMU message

    // ----> Preparing average IMU message
    // This assumes measurements at a constant frequency
    // Also possible: current_timestamp - last_timestamp (requires start timestamp)

    // Filling avg IMU message with data
    avg_imu_msg.header.stamp.sec = burst_timestamp / 1000000000;
    avg_imu_msg.header.stamp.nanosec = burst_timestamp % 1000000000;
    avg_imu_msg.angular_velocity.x = total_rx / number_of_samples;
    avg_imu_msg.angular_velocity.y = total_ry / number_of_samples;
    avg_imu_msg.angular_velocity.z = total_rz / number_of_samples;
    avg_imu_msg.linear_acceleration.x = total_ax / number_of_samples;
    avg_imu_msg.linear_acceleration.y = total_ay / number_of_samples;
    avg_imu_msg.linear_acceleration.z = total_az / number_of_samples;
    // <---- Preparing average IMU message

    // ----> Publishing IMU messages
    imu_burst_pub_->publish(imu_burst_msg);
    avg_imu_pub_->publish(avg_imu_msg);
    // <---- Publishing IMU messages
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARPublisher>("lidar_publisher");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}