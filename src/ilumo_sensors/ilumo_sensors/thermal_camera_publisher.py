#!/usr/bin/env python3

import cv2
import numpy as np
from cv_bridge import CvBridge

from serial import SerialException

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Temperature
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange

from ilumo_interfaces.msg import ThermalImage

from senxor.utils import data_to_frame, connect_senxor, remap

class ThermalCameraPublisher(Node):
    def __init__(self):
        super().__init__("thermal_camera_publisher")

        # ----> Declare the parameters
        fps_param_descriptor = ParameterDescriptor(name = "Framerate",
                                                   type = 2, # Integer
                                                   description = 'Framerate of the thermal camera. (default=15)', 
                                                   integer_range = [IntegerRange(from_value = 1,
                                                                                 to_value = 25)])
        
        updown_param_descriptor = ParameterDescriptor(name = "Upside Down",
                                                      type = 1, # Bool
                                                      description = 'Enable if thermal camera is upside down (USB port is up). (default=True)')

        self.declare_parameter('fps', 15, fps_param_descriptor)
        self.declare_parameter('updown', True, updown_param_descriptor)
        # <---- Declare the parameters

        self.get_logger().info(f'Starting thermal camera node ...')

        # ----> Create the publishers
        self.data_pub_ = self.create_publisher(ThermalImage, "thermal_camera/thermal_data", 10)
        self.img_pub_ = self.create_publisher(Image, "thermal_camera/image", 10)
        self.temp_pub_ = self.create_publisher(Temperature, "thermal_camera/sensor_temperature", 10)
        # <---- Create the publishers

        # ----> Connect to thermal camera
        # Attempts connection to first USB device with vendor ID 1046 and product ID 45058 or 45088
        try:
            self.mi48, connected_port, _ = connect_senxor()
        except SerialException:
            # port already open
            self.get_logger().error(f'Could not connecto to {connected_port.description}. This might mean access was denied. Try running: sudo chmod 666 {connected_port.device}.')

        if self.mi48 is None:
            self.get_logger().error('Connection to thermal camera failed.')

        self.get_logger().info(f'Connected to thermal camera at port {connected_port}.')
        # <---- Connect to thermal camera

        # ----> Set thermal camera parameters
        stream_fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.mi48.set_fps(stream_fps)
        self.last_img_ts = 0

        # see if filtering is available in MI48 and set it up
        self.mi48.disable_filter(f1=True, f2=True, f3=True)
        self.mi48.set_filter_1(85)
        self.mi48.enable_filter(f1=True, f2=False, f3=False, f3_ks_5=False)
        self.mi48.set_offset_corr(0.0)

        self.mi48.set_sens_factor(100)
        self.mi48.get_sens_factor()
        # <---- Set thermal camera parameters

        # ----> Start thermal video stream
        self.mi48.start(stream=True, with_header=True)
        # <---- Start thermal video stream

        # ----> Prepare thermal data message
        self.data_msg = ThermalImage()
        self.data_msg.header.frame_id = 'thermal_camera'
        self.data_msg.height = 62
        self.data_msg.width = 80 
        self.data_msg.step = 80*4
        self.data_msg.encoding = '32FC1'
        # <---- Prepare thermal data message

        # ----> Prepare thermal image message
        self.img_msg = Image()
        self.img_msg.header.frame_id = 'thermal_camera'
        self.img_msg.height = 62
        self.img_msg.width = 80 
        self.img_msg.step = 80 * 3
        self.img_msg.encoding = 'bgr8'
        # <---- Prepare thermal image message

        # ----> Prepare temperature message
        self.temp_msg = Temperature()
        # <---- Prepare temperature message
        
        # ----> Initialize publisher
        self.timer_ = self.create_timer(0.04, self.thermalcameraCallback)
        # <---- Initialize publisher

    def thermalcameraCallback(self):
        # ----> Get camera data and current timestamp
        data, header = self.mi48.read()
        timestamp = self.get_clock().now()
        # <---- Get camera data and current timestamp

        # ----> Thermal camera Debug information
        self.get_logger().debug(f"Thermal timestamp: {timestamp.nanoseconds/1e9}")
        if self.last_img_ts!=0:
            self.get_logger().debug(f" [{1e9/(timestamp.nanoseconds-self.last_img_ts)} Hz]")

        self.last_img_ts = timestamp.nanoseconds

        if data is None:
            self.get_logger().error('Thermal camera received NONE data instead of GFRA')
            return
        # <---- Thermal camera Debug information

        # ----> Rotate image and prepare rgb version
        updown = self.get_parameter('updown').get_parameter_value().bool_value

        if updown:
            frame = np.flip(data_to_frame(data, (80,62), hflip=False), 0)
        else:
            frame = data_to_frame(data, (80,62), hflip=True)

        heatmap = cv2.applyColorMap(remap(frame), cv2.COLORMAP_JET)
        # <---- Rotate image and prepare rgb version

        # ----> Prepare image and data message
        bridge = CvBridge()
        cv_image = bridge.cv2_to_imgmsg(heatmap, encoding='bgr8')

        self.data_msg.data = frame.astype(np.float32).flatten().tolist()
        self.img_msg.data = cv_image.data

        self.data_msg.header.stamp = timestamp.to_msg()
        self.img_msg.header.stamp = timestamp.to_msg()

        self.temp_msg.temperature = header['senxor_temperature']
        self.temp_msg.header.stamp = timestamp.to_msg()
        # <---- Prepare image and data messages

        # ----> Publish messages
        self.data_pub_.publish(self.data_msg)
        self.img_pub_.publish(self.img_msg)
        self.temp_pub_.publish(self.temp_msg)
        # <---- Publish messages


def main():
    rclpy.init()

    simple_publisher = ThermalCameraPublisher()
    rclpy.spin(simple_publisher)
    
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()