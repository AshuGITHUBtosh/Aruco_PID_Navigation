#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToArduino(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_arduino')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.serial_port = serial.Serial('/dev/ttyUSB0', 19200)  # Adjust the port name and baud rate as needed

    def listener_callback(self, msg):
        linear_velocity = msg.linear.x * 1
        angular_velocity = msg.angular.z * 1

        # Convert the velocity commands to PWM valuesc
        left_pwm = int((linear_velocity)+(angular_velocity))*100;  # Scale to range [0, 255]
        right_pwm =int((linear_velocity)-(angular_velocity))*100; # Scale to range [0, 255]
 




        # Constrain the PWM values to be within the valid range
        # left_pwm = max(-255, min(255, left_pwm))
        # right_pwm = max(0, min(255, right_pwm))

        # Send the PWM values to the Arduino
        self.serial_port.write(f"{left_pwm},{right_pwm}\n".encode())
        print(f"{left_pwm},{right_pwm}\n")
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()