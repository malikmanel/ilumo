# Code based on this: https://forums.developer.nvidia.com/t/pwm-not-functioning-on-pins-32-and-33/283373
# Currently based on the TB6612FNG Dual H-Bridge

import rclpy
from rclpy.node import Node
from ilumo_interfaces.srv import SetLedBrightness, SetMotorSpeed

import Jetson.GPIO as GPIO

class PMWController(Node):
    def __init__(self):
        super().__init__("pwm_controller")

        # Set the mode of numbering the pins.
        GPIO.setmode(GPIO.BOARD)

        # Motor pins
        self.pwma = 32  # Speed control for motor A (PWM)
        self.ai1 = 35  # Direction
        self.ai2 = 37  # Direction

        # LED pins
        self.pwmb = 33  # Brightness control for LED (PWM)
        self.bi1 = 38  # Direction
        self.bi2 = 40  # Direction

        # Set up the pins
        GPIO.setup(self.pwma, GPIO.OUT)
        GPIO.setup(self.ai1, GPIO.OUT)
        GPIO.setup(self.ai2, GPIO.OUT)
        GPIO.setup(self.pwmb, GPIO.OUT)
        GPIO.setup(self.bi1, GPIO.OUT)
        GPIO.setup(self.bi2, GPIO.OUT)

        # Set PWM frequency
        self.pwm_motor = GPIO.PWM(self.pwma, 1000)  # Frequency in Hz
        self.pwm_led = GPIO.PWM(self.pwmb, 1000)  # Frequency in Hz

        # Start PWM
        self.pwm_motor.start(0)
        self.pwm_led.start(0)

        self.service_ = self.create_service(SetMotorSpeed, "set_motor_speed", self.motorControlCallback)
        self.service_ = self.create_service(SetLedBrightness, "set_led_brightness", self.ledControlCallback)
        self.get_logger().info("PWM services ready")

        def __del__(self):
            self.pwm_motor.stop()
            self.pwm_led.stop()
            GPIO.cleanup()


    def motorControlCallback(self, req, res):
        # TODO: calculate sduty cycle from req.speed
        self.pwm_motor.ChangeDutyCycle(req.speed)

        # TODO: set depending on motor controller
        if req.speed == 0:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.LOW)
        elif req.forward:
            GPIO.output(self.ai1, GPIO.HIGH)
            GPIO.output(self.ai2, GPIO.LOW)
        else:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.HIGH)

        res.success = True

        return res
    
    def ledControlCallback(self, req, res):
        # TODO: calculate duty cycle from req.brightness
        self.pwm_led.ChangeDutyCycle(req.brightness)

        # TODO: set depending on motor controller
        if req.brightness == 0:
            GPIO.output(self.bi1, GPIO.LOW)
            GPIO.output(self.bi2, GPIO.LOW)
        else:
            GPIO.output(self.bi1, GPIO.HIGH)
            GPIO.output(self.bi2, GPIO.LOW)

        res.success = True

        return res

def main():
    rclpy.init()

    pmw_controller_server = PMWController()
    rclpy.spin(pmw_controller_server)
    
    pmw_controller_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()