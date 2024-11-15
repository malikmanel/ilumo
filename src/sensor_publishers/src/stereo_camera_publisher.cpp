// ----> Includes
#include "sensor_publishers/stereo_camera_publisher.hpp"
#include "zed2_interface/videocapture.hpp"
#include "zed2_interface/sensorcapture.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
// <---- Includes

// ----> Functions
// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap);
// <---- Functions

// ----> Global variables
std::mutex imuMutex;
std::string imuTsStr;
std::string imuAccelStr;
std::string imuGyroStr;

bool sensThreadStop=false;
uint64_t mcu_sync_ts=0;
// <---- Global variables

StereoCameraPublisher::StereoCameraPublisher(const std::string& name) : Node(name)
{
    // publishers for all camera info. Still needs barometer and all that. Also prob. split IMU into multiple publishers
    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/left_camera/image", 10);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/right_camera/image", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("stereo_camera/imu", 10);
    magnetometer_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("stereo_camera/magnetometer", 10);
    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("stereo_camera/environment/pressure", 10);
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/environment/temperature", 10);;
    humidity_pub_ = create_publisher<sensor_msgs::msg::RelativeHumidity>("stereo_camera/environment/humidity", 10);;
    camera_temperature_left_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/left_camera/temperature", 10);;
    camera_temperature_right_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/right_camera/temperature", 10);;

    //sl_oc::sensors::SensorCapture::resetSensorModule();
    //sl_oc::sensors::SensorCapture::resetVideoModule();

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

    // Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
    // minor (max 100Hz), we use a separated thread for sensors.
    std::thread sensThread(getSensorThreadFunc,&sensCap);
    // <---- Create Sensors Capture

    // ----> Enable video/sensors synchronization
    videoCap.enableSensorSync(&sensCap);
    // <---- Enable video/sensors synchronization

    // ----> Prepare Image messages
    auto left_image = sensor_msgs::msg::Image();
    auto right_image = sensor_msgs::msg::Image();

    int w,h;
    videoCap.getFrameSize(w,h);

    // left_image.frame_id = camera frame id;
    left_image.height = h;
    left_image.width = w;
    // right_image.frame_id = camera frame id;
    right_image.height = h;
    right_image.width = w;
    // <---- Prepare Image messages

    uint64_t last_timestamp = 0;

    float frame_fps=0;

    // Infinite grabbing loop
    while (1)
    {
        // ----> Get Video frame
        // Get last available frame
        const sl_oc::video::Frame frame = videoCap.getLastFrame(1);

        // If the frame is valid we can update it
        if(frame.data!=nullptr && frame.timestamp!=last_timestamp)
        {
            frame_fps = 1e9/static_cast<float>(frame.timestamp-last_timestamp);
            last_timestamp = frame.timestamp;
        }
        // <---- Get Video frame

        // ----> Video Debug information
        RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Video timestamp: " << static_cast<double>(last_timestamp)/1e9<< " sec");
        if( last_timestamp!=0 )
            RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << frame_fps << " Hz]");
        // <---- Video Debug information

        // ----> Publish frame data
        if(frame.data!=nullptr)
        {
            // How to split data for right image and left image?
            left_image.data = frame.data;
            left_image.header.stamp = frame.timestamp;
            left_image_pub_->publish(left_image);

            right_image.header.stamp = frame.timestamp;
            right_image.data = frame.data;
            right_image_pub_->publish(right_image);
        }
        // <---- Publish frame data
    }
}

// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap)
{
    // Flag to stop the thread
    sensThreadStop = false;

    // Previous IMU timestamp to calculate frequency
    uint64_t last_imu_ts = 0;

    // Infinite data grabbing loop
    while(!sensThreadStop)
    {
        // ----> Get IMU data
        const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);

        // Process data only if valid
        if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
        {
            // ----> Data info to be displayed
            std::stringstream timestamp;
            std::stringstream accel;
            std::stringstream gyro;

            timestamp << std::fixed << std::setprecision(9) << "IMU timestamp:   " << static_cast<double>(imuData.timestamp)/1e9<< " sec" ;
            if(last_imu_ts!=0)
                timestamp << std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(imuData.timestamp-last_imu_ts) << " Hz]";
            last_imu_ts = imuData.timestamp;

            accel << std::fixed << std::showpos << std::setprecision(4) << " * Accel: " << imuData.aX << " " << imuData.aY << " " << imuData.aZ << " [m/s^2]";
            gyro << std::fixed << std::showpos << std::setprecision(4) << " * Gyro: " << imuData.gX << " " << imuData.gY << " " << imuData.gZ << " [deg/s]";
            // <---- Data info to be displayed

            // Mutex to not overwrite data while diplaying them
            imuMutex.lock();

            imuTsStr = timestamp.str();
            imuAccelStr = accel.str();
            imuGyroStr = gyro.str();

            // ----> Timestamp of the synchronized data
            if(imuData.sync)
            {
                mcu_sync_ts = imuData.timestamp;
            }
            // <---- Timestamp of the synchronized data

            imuMutex.unlock();
        }
        // <---- Get IMU data
    }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraPublisher>("stereo_camera_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}