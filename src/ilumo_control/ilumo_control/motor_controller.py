import rclpy
from rclpy.node import Node
from ilumo_interfaces.srv import SetLedBrightness


class LedController(Node):
    def __init__(self):
        super().__init__("led_controller")
        self.service_ = self.create_service(SetLedBrightness, "set_led_brightness", self.serviceCallback)
        self.get_logger().info("Service set_led_brightness ready")


    def serviceCallback(self, req, res):
        self.get_logger().info("New Request Received a: %d, b: %d" % (req.a, req.b))
        res.sum = req.a + req.b
        self.get_logger().info("Returning sum: %d" % res.sum)
        return res


def main():
    rclpy.init()

    simple_service_server = LedController()
    rclpy.spin(simple_service_server)
    
    simple_service_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()