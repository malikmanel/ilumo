// ----> Includes
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>

#include <sensor_msgs/fill_image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ilumo_sensors/stereo_camera_publisher.hpp"
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
    // ----> Create the publishers
    left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_camera/left_camera/image", 10);
    right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_camera/right_camera/image", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("stereo_camera/imu", 10);
    magnetometer_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("stereo_camera/magnetometer", 10);
    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("stereo_camera/environment/pressure", 10);
    temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/environment/temperature", 10);
    humidity_pub_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("stereo_camera/environment/humidity", 10);
    left_camera_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/left_camera/temperature", 10);
    right_camera_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/right_camera/temperature", 10);
    // <---- Create the publishers

    // Reset Modules
    sl_oc::sensors::SensorCapture::resetSensorModule();
    sl_oc::sensors::SensorCapture::resetVideoModule();

    // Set the verbose level
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

    // ----> Set the video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = verbose;
    // <---- Video parameters

    // ----> Create a Video Capture object
    videoCap = std::make_shared<sl_oc::video::VideoCapture>(params);
    if( !videoCap->initializeVideo(-1) )
    {
        RCLCPP_ERROR(get_logger(), "Cannot open camera video capture");
    }

    // Serial number of the connected camera
    int camSn = videoCap->getSerialNumber();
    RCLCPP_INFO_STREAM(get_logger(), "Video Capture connected to camera sn: " << camSn);
    // <---- Create a Video Capture object

    // ----> Create a Sensors Capture object
    sensCap = std::make_shared<sl_oc::sensors::SensorCapture>(verbose);
    if( !sensCap->initializeSensors(camSn) ) // Note: we use the serial number acquired by the VideoCapture object
    {
        RCLCPP_ERROR(get_logger(), "Cannot open sensors capture");
    }

    RCLCPP_INFO_STREAM(get_logger(), "Sensors Capture connected to camera sn: " << sensCap->getSerialNumber());

    // ----> Enable video/sensors synchronization
    //videoCap->enableSensorSync(sensCap);
    // <---- Enable video/sensors synchronization

    // ----> Prepare Image messages
    left_image_msg = sensor_msgs::msg::Image();
    right_image_msg = sensor_msgs::msg::Image();

    // ---> Get frame size
    videoCap->getFrameSize(width, height);
    width /= 2; // This assumes the camera provides both images as one frame, but I don't know how to split them yet
    // <--- Get frame size

    // left_image_msg.frame_id = camera frame id;
    left_image_msg.height = height;
    left_image_msg.width = width;
    // right_image_msg.frame_id = camera frame id;
    right_image_msg.height = height;
    right_image_msg.width = width;
    // <---- Prepare Image messages

    // ----> Prepare Sensor messages
    imu_msg = sensor_msgs::msg::Imu();
    mag_msg = sensor_msgs::msg::MagneticField();
    press_msg = sensor_msgs::msg::FluidPressure();
    temp_msg = sensor_msgs::msg::Temperature();
    humi_msg = sensor_msgs::msg::RelativeHumidity();
    left_cam_temp_msg = sensor_msgs::msg::Temperature();
    right_cam_temp_msg = sensor_msgs::msg::Temperature();

    // imu_msg.header.frame_id = ...;
    // mag_msg.header.frame_id = ...;
    // press_msg.header.frame_id = ...;
    // temp_msg.header.frame_id = ...;
    // humi_msg.header.frame_id = ...;
    // left_cam_temp_msg.header.frame_id = ...;
    // right_cam_temp_msg.header.frame_id = ...;
    // <---- Prepare Sensor messages

    // Previous timestamps to calculate frequency
    last_img_ts = 0;
    last_imu_ts = 0;
    last_mag_ts = 0;
    last_env_ts = 0;
    last_cam_temp_ts = 0;

    frame_fps = 0;

    callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    image_timer_ = this->create_wall_timer(20ms, std::bind(&StereoCameraPublisher::imageCallback, this), 
                                           callback_group);
    sensor_timer_ = this->create_wall_timer(2ms, std::bind(&StereoCameraPublisher::sensorCallback, this),
                                           callback_group);
}

void StereoCameraPublisher::imageCallback()
{
    // ----> Get Video frame
    const sl_oc::video::Frame frame = videoCap->getLastFrame(10);
    // <---- Get Video frame

    // If the frame is valid we can update it
    if(frame.data!=nullptr && frame.timestamp!=last_img_ts)
    {
        std::cout << "YABAYABAYEEEET" << std::endl;
        frame_fps = 1e9/static_cast<float>(frame.timestamp-last_img_ts);
        last_img_ts = frame.timestamp;

        // ----> Video Debug information
        RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Video timestamp: " << static_cast<double>(last_img_ts)/1e9<< " sec");
        if( last_img_ts!=0 )
            RCLCPP_INFO_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << frame_fps << " Hz]");
        // <---- Video Debug information

        // ----> Conversion from YUV 4:2:2 to BGR for visualization
        cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR(frame.height, frame.height, CV_8UC3, cv::Scalar(0,0,0));
        cv::cvtColor(frameYUV,frameBGR, cv::COLOR_YUV2BGR_YUYV);
        // <---- Conversion from YUV 4:2:2 to BGR for visualization

        // ----> Split image
        cv::Mat left_image = frameBGR(cv::Rect(0, 0, left_image_msg.width, frame.height)).clone();
        cv::Mat right_image = frameBGR(cv::Rect(left_image_msg.width, 0, right_image_msg.width, frame.height)).clone();
        // <---- Split image

        // ----> Preparing image messages
        std_msgs::msg::Header header; // empty header
        // header.frame_id = frame_id;
        header.stamp.nanosec = frame.timestamp;

        sensor_msgs::msg::Image left_img_msg = *cv_bridge::CvImage(header, "bgr8", left_image).toImageMsg();  
        sensor_msgs::msg::Image right_img_msg = *cv_bridge::CvImage(header, "bgr8", right_image).toImageMsg();  
        // <---- Preparing image messages

        // ----> Publishing image messages
        left_image_pub_->publish(left_img_msg);
        right_image_pub_->publish(right_img_msg);
        // <---- Publishing image messages
    }
}

// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void StereoCameraPublisher::sensorCallback()
{
    // ----> Get IMU data
    const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);

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
        imu_msg.linear_acceleration.y = imuData.aY;
        imu_msg.linear_acceleration.z = imuData.aZ;
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
    const sl_oc::sensors::data::Magnetometer magData = sensCap->getLastMagnetometerData(100);

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
    const sl_oc::sensors::data::Environment envData = sensCap->getLastEnvironmentData(100);

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
    const sl_oc::sensors::data::Temperature tempData = sensCap->getLastCameraTemperatureData(100);
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
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}