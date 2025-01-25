import rclpy
from rclpy.node import Node
from ilumo_interfaces.srv import SetMotorSpeed


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.service_ = self.create_service(SetMotorSpeed, "set_motor_speed", self.serviceCallback)
        self.get_logger().info("Service set_motor_speed ready")


    def serviceCallback(self, req, res):
        self.get_logger().info("New Request Received a: %d, b: %d" % (req.a, req.b))
        res.sum = req.a + req.b
        self.get_logger().info("Returning sum: %d" % res.sum)
        return res


def main():
    rclpy.init()

    simple_service_server = MotorController()
    rclpy.spin(simple_service_server)
    
    simple_service_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()