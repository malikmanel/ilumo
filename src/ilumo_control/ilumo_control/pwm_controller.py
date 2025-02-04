# Code based on this: https://forums.developer.nvidia.com/t/pwm-not-functioning-on-pins-32-and-33/283373
# Currently based on the TB6612FNG Dual H-Bridge

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange

from ilumo_interfaces.srv import SetLedBrightness, SetMotorSpeed

import Jetson.GPIO as GPIO

class PMWController(Node):
    def __init__(self):
        super().__init__("pwm_controller")
        
        # ----> Declare the parameters
        led_lumen_param_descriptor = ParameterDescriptor(name = "LED Brightness",
                                                         type = 2, # Integer
                                                         description = 'Total brightness of the LED lamps in lm. (default=800)', 
                                                         integer_range = IntegerRange(from_value = 0,
                                                                                      to_value = 1005))

        self.declare_parameter('led_brightness', 800, led_lumen_param_descriptor)
        # <---- Declare the parameters

        # ----> Prepare the pmw pins
        # Set the mode of numbering the pins.
        GPIO.setmode(GPIO.BOARD)

        # Motor pins
        self.pwm1 = 32  # Speed control for motor (PWM)
        self.pwm2 = 33  # Speed control for motor (PWM)

        # LED pins
        self.pwm3 = 15 # TODO: Find pin # Brightness control for LED (PWM)

        # Set up the pins
        GPIO.setup(self.pwm1, GPIO.OUT)
        GPIO.setup(self.pwm2, GPIO.OUT)
        GPIO.setup(self.pwm3, GPIO.OUT)

        # Set PWM frequency
        self.pwm_motor1 = GPIO.PWM(self.pwm1, 15000)  # Frequency in Hz
        self.pwm_motor2 = GPIO.PWM(self.pwm2, 15000)  # Frequency in Hz
        self.pwm_led = GPIO.PWM(self.pwm3, 10000)  # Frequency in Hz

        # Start PWM
        self.pwm_motor1.start(0)
        self.pwm_motor2.start(0)
        self.pwm_led.start(0)
        # <---- Prepare the pmw pins

        # ----> Turn on lights
        initial_brightness = self.get_parameter('led_brightness').get_parameter_value().integer_value
        self.pwm_led.ChangeDutyCycle(initial_brightness/1005)
        # <---- Turn on lights

        # ----> Initialize motor and LED control services
        self.service_ = self.create_service(SetMotorSpeed, "set_motor_speed", self.motorControlCallback)
        self.service_ = self.create_service(SetLedBrightness, "set_led_brightness", self.ledControlCallback)
        self.get_logger().info("Motor and LED control services available.")
        # <---- Initialize motor and LED control services

    def __del__(self):
        self.pwm_motor.stop()
        self.pwm_led.stop()
        GPIO.cleanup()


    def motorControlCallback(self, req, res):
        # ----> Motor speed limitation
        if req.speed > 8.0:
            req.speed = 8-0
            self.get_logger().warning(f'Requested motor speed exceeded limit of 8 rpm (requested {req.speed} rpm). Reduced to 8 rpm.')
        # <---- Motor speed limitation

        # ----> Changing motor PMW cycle
        if req.speed == 0:
            self.pwm_motor1.ChangeDutyCycle(0)
            self.pwm_motor2.ChangeDutyCycle(0)
        elif req.forward:
            self.pwm_motor1.ChangeDutyCycle(duty_cycle_percent=req.speed/10)
            self.pwm_motor2.ChangeDutyCycle(0)
        else:
            self.pwm_motor1.ChangeDutyCycle(0)
            self.pwm_motor2.ChangeDutyCycle(duty_cycle_percent=req.speed/10)
        # <---- Changing motor PMW cycle

        res.success = True
        return res
    
    def ledControlCallback(self, req, res):
        # ----> LED brightness limitation
        if req.brightness > 1005:
            req.brightness = 1005
            self.get_logger().warning(f'Requested LED brightness exceeded limit of 1005 lm (requested {req.brightness} lm). Reduced to 1005 lm.')
        # <---- LED brightness limitation
        
        # ----> Changing LED PMW cycle
        self.pwm_led.ChangeDutyCycle(req.brightness/1005)
        # <---- Changing LED PMW cycle

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