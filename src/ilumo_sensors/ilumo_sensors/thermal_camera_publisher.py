#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Temperature

from ilumo_interfaces.msg import ThermalImage

from senxor.mi48 import format_header, format_framestats
from senxor.utils import data_to_frame, connect_senxor, remap

class ThermalCameraPublisher(Node):
    def __init__(self):
        super().__init__("thermal_camera_publisher")
        self.data_pub_ = self.create_publisher(ThermalImage, "thermal_camera/thermal_data", 10)
        self.img_pub_ = self.create_publisher(Image, "thermal_camera/image", 10)
        self.temp_pub_ = self.create_publisher(Temperature, "thermal_camera/sensor_temperature", 10)

        # Make an instance of the MI48, attaching USB for 
        # both control and data interface.
        # 'src=0' should refer to the first usb device with 
        # vendor ID 1046 and product ID 45058 or 45088
        self.mi48, connected_port, _ = connect_senxor(src=0)

        if self.mi48 is None:
            self.get_logger().error('Connection to thermal camera failed.')
            exit()

        self.get_logger().info(f'Connected to thermal camera at port {connected_port}.')

        # print out camera infoSensorCapture
        STREAM_FPS = 25
        self.mi48.set_fps(STREAM_FPS)

        # see if filtering is available in MI48 and set it up
        self.mi48.disable_filter(f1=True, f2=True, f3=True)
        self.mi48.set_filter_1(85)
        self.mi48.enable_filter(f1=True, f2=False, f3=False, f3_ks_5=False)
        self.mi48.set_offset_corr(0.0)

        self.mi48.set_sens_factor(100)
        self.mi48.get_sens_factor()

        # initiate continuous frame acquisition
        self.mi48.start(stream=True, with_header=True)

        # Prepare data message
        self.data_msg = ThermalImage()
        # self.img_msg.header.frame_id =
        self.data_msg.height = 62
        self.data_msg.width = 80 
        self.data_msg.step = 80*4
        self.data_msg.encoding = '32FC1'

        # Prepare image message
        self.img_msg = Image()
        # self.img_msg.header.frame_id =
        self.img_msg.height = 62
        self.img_msg.width = 80 
        self.img_msg.step = 80
        self.img_msg.encoding = 'mono8'

        # Prepare temperature message
        self.temp_msg = Temperature()
        
        self.timer_ = self.create_timer(0.04, self.thermalcameraCallback)

    def thermalcameraCallback(self):
        data, header = self.mi48.read()

        if data is None:
            self.get_logger().error('Thermal camera received NONE data instead of GFRA')
            return

        # 
        if header is not None:
            self.get_logger().debug('  '.join([format_header(header), format_framestats(data)]))
        else:
            self.get_logger().debug(format_framestats(data))

        frame = np.flip(data_to_frame(data, (80,62), hflip=False), 1)

        self.data_msg.data = frame.astype(np.float32).flatten().tolist()
        self.img_msg.data = remap(frame).flatten().tolist()

        #self.data_msg.header.stamp = header['timestamp']
        #self.img_msg.header.stamp = header['timestamp']

        self.temp_msg.temperature = header['senxor_temperature']
        #self.temp_msg.header.stamp = header['timestamp']

        self.data_pub_.publish(self.data_msg)
        self.img_pub_.publish(self.img_msg)
        self.temp_pub_.publish(self.temp_msg)


def main():
    rclpy.init()

    simple_publisher = ThermalCameraPublisher()
    rclpy.spin(simple_publisher)
    
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()