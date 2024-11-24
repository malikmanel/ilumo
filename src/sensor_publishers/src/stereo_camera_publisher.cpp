// ----> Includes
#include "sensor_publishers/stereo_camera_publisher.hpp"
#include "zed2_interface/videocapture.hpp"
#include "zed2_interface/sensorcapture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/fill_image.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
// <---- Includes

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
    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/left_camera/image", 10);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo_camera/right_camera/image", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("stereo_camera/imu", 10);
    magnetometer_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("stereo_camera/magnetometer", 10);
    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("stereo_camera/environment/pressure", 10);
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/environment/temperature", 10);;
    humidity_pub_ = create_publisher<sensor_msgs::msg::RelativeHumidity>("stereo_camera/environment/humidity", 10);;
    camera_temperature_left_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/left_camera/temperature", 10);;
    camera_temperature_right_pub_ = create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/right_camera/temperature", 10);;

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
    //sl_oc::sensors::SensorCapture sensCap(verbose);
    //if( !sensCap.initializeSensors(camSn) ) // Note: we use the serial number acquired by the VideoCapture object
    //{
    //    RCLCPP_ERROR(get_logger(), "Cannot open sensors capture");
    //    rclcpp::shutdown();
    //}

    //RCLCPP_INFO_STREAM(get_logger(), "Sensors Capture connected to camera sn: " << sensCap.getSerialNumber());

    // Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
    // minor (max 100Hz), we use a separated thread for sensors.
    // std::thread sensThread(StereoCameraPublisher::getSensorThreadFunc,&sensCap);
    // <---- Create Sensors Capture

    // ----> Enable video/sensors synchronization
    //videoCap.enableSensorSync(&sensCap);
    // <---- Enable video/sensors synchronization

    // ----> Prepare Image messages
    auto left_image_msg = sensor_msgs::msg::Image();
    auto right_image_msg = sensor_msgs::msg::Image();

    // ---> Get frame size
    int width,height;
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
        //RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Video timestamp: " << static_cast<double>(last_timestamp)/1e9<< " sec");
        //if( last_timestamp!=0 )
        //    RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << frame_fps << " Hz]");
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
}

// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void StereoCameraPublisher::getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap)
{
    // ----> Create IMU message template
    auto imu_msg = sensor_msgs::msg::Imu();
    // imu_msg.header.frame_id = ...;
    // <---- Create IMU message template

    // Previous IMU timestamp to calculate frequency, See IMU Debug Info
    // uint64_t last_imu_ts = 0;

    // Infinite data grabbing loop
    while(1)
    {
        // ----> Get IMU data
        const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);

        // Process data only if valid
        if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
        {
            // ----> IMU Debug information, Disabled for now as I need to figure out how to do this in a static member function
            // RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "IMU timestamp:   " << static_cast<double>(imuData.timestamp)/1e9<< " sec" );
            // if(last_imu_ts!=0)
            //     RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(imuData.timestamp-last_imu_ts) << " Hz]");
            // last_imu_ts = imuData.timestamp;
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