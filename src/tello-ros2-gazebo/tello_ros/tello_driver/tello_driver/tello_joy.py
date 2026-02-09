import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tello_msgs.srv import TelloAction


class TelloJoyNode(Node):
    def __init__(self):
        super().__init__('tello_joy')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tello_client = self.create_client(TelloAction, 'tello_action')

        # Default mappings match the previous C++ implementation (Xbox One)
        self.axis_throttle = self.declare_parameter('axis_throttle', 4).value
        self.axis_strafe = self.declare_parameter('axis_strafe', 3).value
        self.axis_vertical = self.declare_parameter('axis_vertical', 1).value
        self.axis_yaw = self.declare_parameter('axis_yaw', 0).value
        self.button_takeoff = self.declare_parameter('button_takeoff', 7).value
        self.button_land = self.declare_parameter('button_land', 6).value

        self.create_subscription(Joy, 'joy', self._joy_callback, 10)

    def _send_action(self, cmd):
        if not self.tello_client.service_is_ready():
            self.get_logger().warn('tello_action service not ready')
            return

        req = TelloAction.Request()
        req.cmd = cmd
        self.tello_client.call_async(req)

    @staticmethod
    def _get_axis(msg, idx):
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx])

    @staticmethod
    def _get_button(msg, idx):
        if idx < 0 or idx >= len(msg.buttons):
            return 0
        return int(msg.buttons[idx])

    def _joy_callback(self, msg):
        if self._get_button(msg, self.button_takeoff):
            self._send_action('takeoff')
            return
        if self._get_button(msg, self.button_land):
            self._send_action('land')
            return

        twist = Twist()
        twist.linear.x = self._get_axis(msg, self.axis_throttle)
        twist.linear.y = self._get_axis(msg, self.axis_strafe)
        twist.linear.z = self._get_axis(msg, self.axis_vertical)
        twist.angular.z = self._get_axis(msg, self.axis_yaw)
        self.cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    node = TelloJoyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
