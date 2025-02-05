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
    // ----> Declare the parameters
    // Stereo camera resolution
    auto resolution_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    resolution_param_desc.name = "Resolution";
    resolution_param_desc.type = 2; // Integer
    resolution_param_desc.description = "Resolution of the stereo camera. 0 = VGA, 1 = HD720, 2 = HD1080, 3 = HD2K (default=1)";
    resolution_param_desc.integer_range = {rcl_interfaces::msg::IntegerRange()
                                           .set__from_value(0)
                                           .set__to_value(3)
                                           .set__step(1)};

    // Stereo camera FPS
    auto fps_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    fps_param_desc.name = "Framerate";
    fps_param_desc.type = 2; // Integer
    fps_param_desc.description = "Framerate of the stereo camera. 0 = 15fps, 1 = 30fps, 2 = 60fps, 3 = 100fps (default=1)";
    fps_param_desc.integer_range = {rcl_interfaces::msg::IntegerRange()
                                    .set__from_value(0)
                                    .set__to_value(3)
                                    .set__step(1)};

    // Stereo camera node logging verbosity
    auto logging_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    logging_param_desc.name = "Verbosity";
    logging_param_desc.type = 2; // Integer
    logging_param_desc.description = "Logging verbosity of the stereo camera node. 0 = None, 1 = Error, 2 = Warning, 3 = Info (default=2)";
    logging_param_desc.integer_range = {rcl_interfaces::msg::IntegerRange()
                                        .set__from_value(0)
                                        .set__to_value(3)
                                        .set__step(1)};

    this->declare_parameter("resolution", 1, resolution_param_desc);
    this->declare_parameter("fps", 1, fps_param_desc);
    this->declare_parameter("verbosity", 2, logging_param_desc);
    // <---- Declare the parameters

    RCLCPP_INFO_STREAM(get_logger(), "Starting stereo camera node ...");

    // ----> Create the publishers
    left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_camera/left_camera/image", 10);
    right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_camera/right_camera/image", 10);
    stereo_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_camera/stereo_image", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("stereo_camera/imu", 10);
    magnetometer_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("stereo_camera/magnetometer", 10);
    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("stereo_camera/environment/pressure", 10);
    temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/environment/temperature", 10);
    humidity_pub_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("stereo_camera/environment/humidity", 10);
    left_camera_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/left_camera/temperature", 10);
    right_camera_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("stereo_camera/right_camera/temperature", 10);
    // <---- Create the publishers

    // ----> Set the video parameters
    sl_oc::video::VideoParams params;
    int resolution_param = this->get_parameter("resolution").as_int();
    int fps_param = this->get_parameter("fps").as_int();
    int verbose_param = this->get_parameter("verbosity").as_int();

    sl_oc::VERBOSITY verbose;
    switch (verbose_param) {
        case 0: {verbose = sl_oc::VERBOSITY::NONE;}
            break;
        case 1: {verbose = sl_oc::VERBOSITY::ERROR;}
            break;
        case 2: {verbose = sl_oc::VERBOSITY::WARNING;}
            break;
        case 3: {verbose = sl_oc::VERBOSITY::INFO;}
            break;
    }
    params.verbose = verbose;

    switch (resolution_param) {
        case 0: {params.res = sl_oc::video::RESOLUTION::VGA;}
            break;
        case 1: {params.res = sl_oc::video::RESOLUTION::HD720;
            if (fps_param > 2) {
                fps_param = 2;
                RCLCPP_WARN_STREAM(get_logger(), "Selected framerate not available at HD720. Throttling to 60fps.");
            }
        }break;
        case 2: {params.res = sl_oc::video::RESOLUTION::HD1080;
            if (fps_param > 1) {
                fps_param = 1;
                RCLCPP_WARN_STREAM(get_logger(), "Selected framerate not available at HD1080. Throttling to 30fps.");
            }
        }break;
        case 3: {params.res = sl_oc::video::RESOLUTION::HD2K;
            if (fps_param > 0) {
                fps_param = 1;
                RCLCPP_WARN_STREAM(get_logger(), "Selected framerate not available at HD2K. Throttling to 15fps.");
            }
        }break;
    }

    switch (fps_param) {
        case 0: {params.fps = sl_oc::video::FPS::FPS_15;}
            break;
        case 1: {params.fps = sl_oc::video::FPS::FPS_30;}
            break;
        case 2: {params.fps = sl_oc::video::FPS::FPS_60;}
            break;
        case 3: {params.fps = sl_oc::video::FPS::FPS_100;}
            break;
    }
    // <---- Set the video parameters

    // ----> Create a Video Capture object
    videoCap = std::make_shared<sl_oc::video::VideoCapture>(params);
    if( !videoCap->initializeVideo(-1) )
    {
        RCLCPP_ERROR(get_logger(), "Cannot open camera video capture");
    }

    // Serial number of the connected camera
    int camSn = videoCap->getSerialNumber();
    RCLCPP_INFO_STREAM(get_logger(), "Video Capture connected to camera sn: " << camSn << " [" << videoCap->getDeviceName() << "]");
    // <---- Create a Video Capture object

    // ----> Create a Sensors Capture object
    sensCap = std::make_shared<sl_oc::sensors::SensorCapture>(verbose);
    if( !sensCap->initializeSensors(camSn) ) // Note: we use the serial number acquired by the VideoCapture object
    {
        RCLCPP_ERROR(get_logger(), "Cannot open sensors capture");
    }

    RCLCPP_INFO_STREAM(get_logger(), "Sensors Capture connected to camera sn: " << sensCap->getSerialNumber());

    // ----> Prepare Sensor messages
    imu_msg = sensor_msgs::msg::Imu();
    mag_msg = sensor_msgs::msg::MagneticField();
    press_msg = sensor_msgs::msg::FluidPressure();
    temp_msg = sensor_msgs::msg::Temperature();
    humi_msg = sensor_msgs::msg::RelativeHumidity();
    left_cam_temp_msg = sensor_msgs::msg::Temperature();
    right_cam_temp_msg = sensor_msgs::msg::Temperature();

    imu_msg.header.frame_id = "stereo_camera_center";
    mag_msg.header.frame_id = "stereo_camera_center";
    press_msg.header.frame_id = "stereo_camera_center";
    temp_msg.header.frame_id = "stereo_camera_center";
    humi_msg.header.frame_id = "stereo_camera_center";
    left_cam_temp_msg.header.frame_id = "stereo_camera_bottom";
    right_cam_temp_msg.header.frame_id = "stereo_camera_top";
    // <---- Prepare Sensor messages

    // ----> Set previous timestamps to calculate frequency (debugging)
    last_img_ts = 0;
    last_imu_ts = 0;
    last_mag_ts = 0;
    last_env_ts = 0;
    last_cam_temp_ts = 0;
    // <---- Set previous timestamps to calculate frequency

    // ----> Initialize publishers
    callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    image_timer_ = this->create_wall_timer(20ms, std::bind(&StereoCameraPublisher::imageCallback, this), 
                                           callback_group);
    sensor_timer_ = this->create_wall_timer(2ms, std::bind(&StereoCameraPublisher::sensorCallback, this),
                                           callback_group);
    // <---- Initialize publishers
}

void StereoCameraPublisher::imageCallback()
{
    // ----> Get Video frame
    const sl_oc::video::Frame frame = videoCap->getLastFrame(10);
    // <---- Get Video frame

    // Publish frame data if the frame is valid
    if(frame.data!=nullptr && frame.timestamp!=last_img_ts){
        float frame_fps = 1e9/static_cast<float>(frame.timestamp-last_img_ts);
        last_img_ts = frame.timestamp;

        // ----> Video Debug information
        RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Video timestamp: " << static_cast<double>(last_img_ts)/1e9<< " sec");
        if( last_img_ts!=0 )
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << frame_fps << " Hz]");
        // <---- Video Debug information

        // ----> Conversion from YUV 4:2:2 to BGR for visualization
        cv::Mat frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
        cv::Mat frameBGR;
        cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
        // <---- Conversion from YUV 4:2:2 to BGR for visualization

        // ----> Split image
        cv::Mat left_image = frameBGR(cv::Rect(0, 0, frame.width/2, frame.height)).clone();
        cv::Mat right_image = frameBGR(cv::Rect(frame.width/2, 0, frame.width/2, frame.height)).clone();
        // <---- Split image

        // ----> Rotate image
        cv::rotate(left_image, left_image, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(right_image, right_image, cv::ROTATE_90_CLOCKWISE);
        // <---- Rotate image

        // ----> Preparing image messages
        std_msgs::msg::Header left_img_header;
        std_msgs::msg::Header right_img_header;
        std_msgs::msg::Header stereo_img_header;

        left_img_header.frame_id = "stereo_camera_bottom";
        left_img_header.stamp.sec = frame.timestamp / 1000000000;
        left_img_header.stamp.nanosec = frame.timestamp % 1000000000;

        right_img_header.frame_id = "stereo_camera_top";
        right_img_header.stamp.sec = frame.timestamp / 1000000000;
        right_img_header.stamp.nanosec = frame.timestamp % 1000000000;

        stereo_img_header.frame_id = "stereo_camera_center";
        stereo_img_header.stamp.sec = frame.timestamp / 1000000000;
        stereo_img_header.stamp.nanosec = frame.timestamp % 1000000000;

        sensor_msgs::msg::Image left_img_msg = *cv_bridge::CvImage(left_img_header, "bgr8", left_image).toImageMsg();  
        sensor_msgs::msg::Image right_img_msg = *cv_bridge::CvImage(right_img_header, "bgr8", right_image).toImageMsg();  
        sensor_msgs::msg::Image stereo_img_msg = *cv_bridge::CvImage(stereo_img_header, "bgr8", frameBGR).toImageMsg();  
        // <---- Preparing image messages

        // ----> Publishing image messages
        left_image_pub_->publish(left_img_msg);
        right_image_pub_->publish(right_img_msg);
        stereo_image_pub_->publish(stereo_img_msg);
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
        RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(9) << "IMU timestamp:   " << static_cast<double>(imuData.timestamp)/1e9<< " sec" );
        if(last_imu_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(imuData.timestamp-last_imu_ts) << " Hz]");
        last_imu_ts = imuData.timestamp;
        // <---- IMU Debug information

        // ----> Prepare IMU message
        imu_msg.header.stamp.sec = imuData.timestamp / 1000000000;
        imu_msg.header.stamp.nanosec = imuData.timestamp % 1000000000;
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
        RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Magnetometer timestamp: " << static_cast<double>(magData.timestamp)/1e9<< " sec" );
        if(last_mag_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(magData.timestamp-last_mag_ts) << " Hz]");
        last_mag_ts = magData.timestamp;
        // ---- Magnetometer Debug information

        // ----> Prepare Magnetometer message
        mag_msg.header.stamp.sec = magData.timestamp / 1000000000;
        mag_msg.header.stamp.nanosec = magData.timestamp % 1000000000;
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
        RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Environment data timestamp: " << static_cast<double>(envData.timestamp)/1e9<< " sec" );
        if(last_env_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(envData.timestamp-last_env_ts) << " Hz]");
        last_env_ts = envData.timestamp;
        // ---- Environment data Debug information

        // ----> Prepare Environment data messages
        int sec = envData.timestamp / 1000000000;
        int nsec = envData.timestamp % 1000000000;

        press_msg.header.stamp.sec = sec;
        press_msg.header.stamp.nanosec = nsec;
        press_msg.header.stamp.nanosec = envData.timestamp;
        press_msg.fluid_pressure = envData.press;

        temp_msg.header.stamp.sec = sec;
        temp_msg.header.stamp.nanosec = nsec;
        temp_msg.header.stamp.nanosec = envData.timestamp;
        temp_msg.temperature = envData.temp;

        humi_msg.header.stamp.sec = sec;
        humi_msg.header.stamp.nanosec = nsec;
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
        RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(9) << "Camera Temperature timestamp: " << static_cast<double>(tempData.timestamp)/1e9<< " sec" );
        if(last_cam_temp_ts!=0)
            RCLCPP_DEBUG_STREAM(get_logger(), std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(tempData.timestamp-last_cam_temp_ts) << " Hz]");
        last_cam_temp_ts = tempData.timestamp;
        // ---- Camera Temperature Debug information

        // ----> Prepare Camera Temperature message
        int sec = tempData.timestamp / 1000000000;
        int nsec = tempData.timestamp % 1000000000;

        left_cam_temp_msg.header.stamp.sec = sec;
        left_cam_temp_msg.header.stamp.nanosec = nsec;
        left_cam_temp_msg.temperature = tempData.temp_left;

        right_cam_temp_msg.header.stamp.sec = sec;
        right_cam_temp_msg.header.stamp.nanosec = nsec;
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