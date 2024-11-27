// ----> Includes
#include <chrono>

#include "ilumo_sensors/lidar_publisher.hpp"
// <---- Includes

using namespace std::chrono_literals;

LiDARPublisher::LiDARPublisher(const std::string& name) : Node(name)
{
    std::string scanner_ip_or_host = "localhost";

    // Create a connection to the device.
    std::shared_ptr<blickfeld::scanner> scanner = blickfeld::scanner::connect(scanner_ip_or_host);
    std::cout << "Connected." << std::endl;

    // Create a pointcloud stream object to receive pointclouds
	auto point_cloud_stream = scanner->get_point_cloud_stream();
    auto imu_stream = scanner->get_imu_stream();

    // Initialize publishers
    point_cloud_timer_ = create_wall_timer(15ms, std::bind(&LiDARPublisher::pointcloudCallback, this));
    imu_timer_ = create_wall_timer(0.8ms, std::bind(&LiDARPublisher::imuCallback, this));
}

void LiDARPublisher::pointcloudCallback()
{
    const blickfeld::protocol::data::Frame frame = point_cloud_stream->recv_frame();

	// Iterate through all the scanlines in a frame
    for (int s_ind = 0; s_ind < frame.scanlines_size(); s_ind++) {

        // also relevant: frame.scanlines(s_ind)

        // Iterate through all the points in a scanline
        for (int p_ind = 0; p_ind < frame.scanlines(s_ind).points_size(); p_ind++) {
            auto& point = frame.scanlines(s_ind).points(p_ind);

            // also relevant: point.start_offset_ns

            // Iterate through all the returns for each points
            // this might not be necessary, maybe the first return is enough
            for (int r_ind = 0; r_ind < point.returns_size(); r_ind++) {
                auto& ret = point.returns(r_ind);
                printf("coordinates: (%f, %f, %f)\n", ret.cartesian(0), ret.cartesian(1), ret.cartesian(2));

                // also relevant: ret.intensity()

            }
        }
    }
}

void LiDARPublisher::imuCallback()
{
    const blickfeld::protocol::data::IMU data = imu_stream->recv_burst();
    // received as bursts of data, so a lower frequency is possible. how do I know how large a burst ist?
    for (auto sample : data.samples()) {
			std::cout << "- acc: ["
				  << std::setprecision(3) << std::fixed
				  << sample.acceleration(0) << ", "
				  << sample.acceleration(1) << ", "
				  << sample.acceleration(2) << "], gyro: ["
				  << sample.angular_velocity(0) << ", "
				  << sample.angular_velocity(1) << ", "
				  << sample.angular_velocity(2) << "]>"
				  << std::endl;
		}
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARPublisher>("lidar_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}