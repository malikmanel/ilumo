#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from senxor.mi48 import format_header, format_framestats
from senxor.utils import data_to_frame, connect_senxor

class ThermalCameraPublisher(Node):
    def __init__(self):
        super().__init__("thermal_camera_publisher")
        self.pub_ = self.create_publisher(Image, "thermal_camera/image", 10)

        # Make an instance of the MI48, attaching USB for 
        # both control and data interface.
        # can try connect_senxor(src='/dev/ttyS3') or similar if default cannot be found
        self.mi48, connected_port, port_names = connect_senxor(src='dev/bus/usb/003/024')

        if self.mi48 is None:
            self.get_logger().error(f'Connection to thermal camera failed.')
            exit()

        # print out camera info
        self.get_logger().info(f'Thermal camera info: {self.mi48.camera_info}')
        self.get_logger().debug(f'Connected port: {connected_port} with name {port_names}')

        # set desired FPS
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
        with_header = True
        self.mi48.start(stream=True, with_header=with_header)

        # Prepare image message
        self.msg = Image()
        # msg.header.frame_id =
        self.msg.height = 62
        self.msg.width = 80 
        
        self.timer_ = self.create_timer(0.04, self.thermalcameraCallback)

    def thermalcameraCallback(self):
        data, header = self.mi48.read()

        if data is None:
            self.get_logger().error('Thermal camera received NONE data received instead of GFRA')
            self.mi48.stop()
            return

        # 
        if header is not None:
            self.get_logger().debug('  '.join([format_header(header), format_framestats(data)]))
        else:
            self.get_logger().debug(format_framestats(data))

        self.get_logger().info(f'Thermal camera temperature: {header["senxor_temperature"]}')

        frame = data_to_frame(data, (80,62), hflip=False)

        self.msg.header.stamp = header['timestamp']
        self.msg.data = frame

        self.pub_.publish(self.msg)


def main():
    rclpy.init()

    simple_publisher = ThermalCameraPublisher()
    rclpy.spin(simple_publisher)
    
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()