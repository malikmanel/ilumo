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
	auto stream = scanner->get_point_cloud_stream();
    point_cloud_timer_ = create_wall_timer(2ms, std::bind(&LiDARPublisher::pointcloudCallback, this));
}

void LiDARPublisher::pointcloudCallback()
{
    const blickfeld::protocol::data::Frame frame = stream->recv_frame();

		/*
		 * Print information about this frame
		 *
		 * Reference implementation of stream operator to clarify available attribute methods:
		 *      std::ostream& operator<<(std::ostream &strm, const blickfeld::protocol::data::Frame& frame) {
		 *              return strm << "<Blickfeld Frame " << frame.id() << ": " << frame.total_number_of_returns() << " returns, "
		 *                      << setprecision(1) << fixed << frame.scan_pattern().horizontal().fov() * 180.0f / M_PI << "x" << frame.scan_pattern().vertical().fov() * 180.0f / M_PI << " FoV, "
		 *                      << setprecision(0) << fixed << frame.scanlines_size() << " scanlines>";
		 *      }
		 */
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARPublisher>("lidar_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}