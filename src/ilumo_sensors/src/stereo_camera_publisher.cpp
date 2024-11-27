// ----> Includes
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>

#include <sensor_msgs/fill_image.hpp>

#include "ilumo_sensors/stereo_camera_publisher.hpp"
#include "zed2_interface/videocapture.hpp"
#include "zed2_interface/sensorcapture.hpp"
// <---- Includes

using namespace std::chrono_literals;

// ----> Functions
// Sensor acquisition runs at 400Hz, so it must be executed in a different thread

// ----> Global variables
std::mutex imuMutex;
std::string imuTsStr;
std::string imuAccelStr;
std::string imuGyroStr;

uint64_t mcu_sync_ts=0;
// <---- Global variables

StereoCameraPublisher::StereoCameraPublisher(const std::string& name) : Node(name)
{
    // publishers for all camera info. Still needs barometer and all that. Also prob. split IMU into multiple publishers
    left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_camera/left_camera/image", 10);
    right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_camera/right_camera/image", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("stereo_camera/imu", 10);
    magnetometer_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("stereo_camera/magnetometer", 10);
    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("stereo_camera/environment/pressure", 10);
    temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/environment/temperature", 10);;
    humidity_pub_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("stereo_camera/environment/humidity", 10);;
    left_camera_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/left_camera/temperature", 10);;
    right_camera_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/right_camera/temperature", 10);;

    sl_oc::sensors::SensorCapture::resetSensorModule();
    sl_oc::sensors::SensorCapture::resetVideoModule();

    // Set the verbose level
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::ERROR;

    // ----> Set the video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = verbose;
    // <---- Video parameters

    // ----> Create a Video Capture object
    sl_oc::video::VideoCapture videoCap(params);
    if( !videoCap.initializeVideo(-1) )
    {
        RCLCPP_ERROR(get_logger(), "Cannot open camera video capture");
        rclcpp::shutdown();
    }

    // Serial number of the connected camera
    int camSn = videoCap.getSerialNumber();
    RCLCPP_INFO_STREAM(get_logger(), "Video Capture connected to camera sn: " << camSn);
    // <---- Create a Video Capture object

    // ----> Create a Sensors Capture object
    sl_oc::sensors::SensorCapture sensCap(verbose);
    if( !sensCap.initializeSensors(camSn) ) // Note: we use the serial number acquired by the VideoCapture object
    {
        RCLCPP_ERROR(get_logger(), "Cannot open sensors capture");
        rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(get_logger(), "Sensors Capture connected to camera sn: " << sensCap.getSerialNumber());

    // ----> Enable video/sensors synchronization
    videoCap.enableSensorSync(&sensCap);
    // <---- Enable video/sensors synchronization

    // ----> Prepare Image messages
    auto left_image_msg = sensor_msgs::msg::Image();
    auto right_image_msg = sensor_msgs::msg::Image();

    // ---> Get frame size
    videoCap.getFrameSize(width,height);
    // width /= 2; // This assumes the camera provides both images as one frame, but I don't know how to split them yet
    // <--- Get frame size

    // left_image_msg.frame_id = camera frame id;
    left_image_msg.height = height;
    left_image_msg.width = width;
    // right_image_msg.frame_id = camera frame id;
    right_image_msg.height = height;
    right_image_msg.width = width;
    // <---- Prepare Image messages

    // ----> Prepare Sensor messages
    auto imu_msg = sensor_msgs::msg::Imu();
    auto mag_msg = sensor_msgs::msg::MagneticField();
    auto press_msg = sensor_msgs::msg::FluidPressure();
    auto temp_msg = sensor_msgs::msg::Temperature();
    auto humi_msg = sensor_msgs::msg::RelativeHumidity();
    auto left_cam_temp_msg = sensor_msgs::msg::Temperature();
    auto right_cam_temp_msg = sensor_msgs::msg::Temperature();

    // imu_msg.header.frame_id = ...;
    // mag_msg.header.frame_id = ...;
    // press_msg.header.frame_id = ...;
    // temp_msg.header.frame_id = ...;
    // humi_msg.header.frame_id = ...;
    // left_cam_temp_msg.header.frame_id = ...;
    // right_cam_temp_msg.header.frame_id = ...;
    // <---- Prepare Sensor messages

    // Previous timestamps to calculate frequency
    uint64_t last_img_ts = 0;
    uint64_t last_imu_ts = 0;
    uint64_t last_mag_ts = 0;
    uint64_t last_env_ts = 0;
    uint64_t last_cam_temp_ts = 0;

    float frame_fps=0;

    // image_timer_ = create_wall_timer(20ms, std::bind(&StereoCameraPublisher::imageCallback, this));
    sensor_timer_ = create_wall_timer(2ms, std::bind(&StereoCameraPublisher::sensorCallback, this));
}

void StereoCameraPublisher::imageCallback()
{
    // ----> Get Video frame
    // Get last available frame
    const sl_oc::video::Frame frame = videoCap.getLastFrame(1);

    // If the frame is valid we can update it
    if(frame.data!=nullptr && frame.timestamp!=last_img_ts)
    {
        frame_fps = 1e9/static_cast<float>(frame.timestamp-last_img_ts);
        last_img_ts = frame.timestamp;
    }
    // <---- Get Video frame

    // ----> Video Debug information
    RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Video timestamp: " << static_cast<double>(last_img_ts)/1e9<< " sec");
    if( last_img_ts!=0 )
        RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << frame_fps << " Hz]");
    // <---- Video Debug information

    // ----> Publish frame data
    if(frame.data!=nullptr)
    {
        // How to split data for right image and left image?
        // Images arrive in YUV 4:2:2
        sensor_msgs::fillImage(left_image_msg,
                        sensor_msgs::image_encodings::YUV422,
                        height, // height
                        width, // width
                        width * sizeof(uint8_t) * 2, // stepSize = width*byte_depth*num_channels
                        frame.data);
        left_image_msg.header.stamp.nanosec = frame.timestamp;
        left_image_pub_->publish(left_image_msg);

        sensor_msgs::fillImage(right_image_msg,
                        sensor_msgs::image_encodings::YUV422,
                        height, // height
                        width, // width
                        width * sizeof(uint8_t) * 2, // stepSize
                        frame.data);
        right_image_msg.header.stamp.nanosec = frame.timestamp;
        right_image_pub_->publish(right_image_msg);
    }
    // <---- Publish frame data
}

// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void StereoCameraPublisher::sensorCallback()
{
    // ----> Get IMU data
    const sl_oc::sensors::data::Imu imuData = sensCap.getLastIMUData(2000);

    // Process data only if valid
    if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
    {
        // ----> IMU Debug information
        RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "IMU timestamp:   " << static_cast<double>(imuData.timestamp)/1e9<< " sec" );
        if(last_imu_ts!=0)
            RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(imuData.timestamp-last_imu_ts) << " Hz]");
        last_imu_ts = imuData.timestamp;
        // ---- IMU Debug information

        // ----> Prepare IMU message
        imu_msg.header.stamp.nanosec = imuData.timestamp;
        // for imu_msg.orientation.x/y/z/w maybe use http://wiki.ros.org/imu_filter_madgwick
        imu_msg.linear_acceleration.x = imuData.aX;
        imu_msg.linear_acceleration.x = imuData.aY;
        imu_msg.linear_acceleration.x = imuData.aZ;
        imu_msg.angular_velocity.x = imuData.gX;
        imu_msg.angular_velocity.y = imuData.gY;
        imu_msg.angular_velocity.z = imuData.gZ;
        // <---- Prepare IMU message

        // ---> Publish IMU message
        imu_pub_->publish(imu_msg);
        // <--- Publish IMU message
    }
    // <---- Get IMU data

    // ----> Get Magnetometer data with a timeout of 100 microseconds to not slow down fastest data (IMU)
    const sl_oc::sensors::data::Magnetometer magData = sensCap.getLastMagnetometerData(100);

    // Process data only if valid
    if( magData.valid == sl_oc::sensors::data::Magnetometer::NEW_VAL )
    {
        // ----> Magnetometer Debug information
        RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Magnetometer timestamp: " << static_cast<double>(magData.timestamp)/1e9<< " sec" );
        if(last_mag_ts!=0)
            RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(magData.timestamp-last_mag_ts) << " Hz]");
        last_mag_ts = magData.timestamp;
        // ---- Magnetometer Debug information

        // ----> Prepare Magnetometer message
        mag_msg.header.stamp.nanosec = magData.timestamp;
        mag_msg.magnetic_field.x = magData.mX;
        mag_msg.magnetic_field.y = magData.mY;
        mag_msg.magnetic_field.z = magData.mZ;
        // <---- Prepare Magnetometer message

        // ---> Publish Magnetometer message
        magnetometer_pub_->publish(mag_msg);
        // <--- Publish Magnetometer message
    }
    // <---- Get Magnetometer data with a timeout of 100 microseconds to not slow down fastest data (IMU)

    // ----> Get Environment data with a timeout of 100 microseconds to not slow down fastest data (IMU)
    const sl_oc::sensors::data::Environment envData = sensCap.getLastEnvironmentData(100);

    // Process data only if valid
    if( envData.valid == sl_oc::sensors::data::Environment::NEW_VAL )
    {
        // ----> Environment data Debug information
        RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Environment data timestamp: " << static_cast<double>(envData.timestamp)/1e9<< " sec" );
        if(last_env_ts!=0)
            RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(envData.timestamp-last_env_ts) << " Hz]");
        last_env_ts = envData.timestamp;
        // ---- Environment data Debug information

        // ----> Prepare Environment data messages
        press_msg.header.stamp.nanosec = envData.timestamp;
        press_msg.fluid_pressure = envData.press;
        temp_msg.header.stamp.nanosec = envData.timestamp;
        temp_msg.temperature = envData.temp;
        humi_msg.header.stamp.nanosec = envData.timestamp;
        humi_msg.relative_humidity = envData.humid;
        // <---- Prepare Environment data messages

        // ---> Publish Environment data messages
        pressure_pub_->publish(press_msg);
        temperature_pub_->publish(temp_msg);
        humidity_pub_->publish(humi_msg);
        // <--- Publish Environment data messages
    }
    // <---- Get Environment data with a timeout of 100 microseconds to not slow down fastest data (IMU)

    // ----> Get Camera temperature data with a timeout of 100 microseconds to not slow down fastest data (IMU)

    // Process data only if valid
    const sl_oc::sensors::data::Temperature tempData = sensCap.getLastCameraTemperatureData(100);
    if( tempData.valid == sl_oc::sensors::data::Temperature::NEW_VAL )
    {
        // ----> Camera Temperature Debug information
        RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Camera Temperature timestamp: " << static_cast<double>(tempData.timestamp)/1e9<< " sec" );
        if(last_cam_temp_ts!=0)
            RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(tempData.timestamp-last_cam_temp_ts) << " Hz]");
        last_cam_temp_ts = tempData.timestamp;
        // ---- Camera Temperature Debug information

        // ----> Prepare Camera Temperature message
        left_cam_temp_msg.header.stamp.nanosec = tempData.timestamp;
        left_cam_temp_msg.temperature = tempData.temp_left;
        right_cam_temp_msg.header.stamp.nanosec = tempData.timestamp;
        right_cam_temp_msg.temperature = tempData.temp_right;
        // <---- Prepare Camera Temperature message

        // ---> Publish Camera Temperature message
        left_camera_temperature_pub_->publish(left_cam_temp_msg);
        right_camera_temperature_pub_->publish(right_cam_temp_msg);
        // <--- Publish Camera Temperature message
    }
    // <---- Get Camera Temperature data with a timeout of 100 microseconds to not slow down fastest data (IMU)
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraPublisher>("stereo_camera_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}