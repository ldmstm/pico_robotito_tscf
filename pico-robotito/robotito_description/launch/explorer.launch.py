import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

class RoomExplorer(Node):
    def __init__(self):
        super().__init__('room_explorer')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.turn_dir = 1.0
        self.obstacle_close = False

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

    def scan_cb(self, msg: LaserScan):
        if len(msg.ranges) < 6:
            return
        sensor_frente = [msg.ranges[0], msg.ranges[1], msg.ranges[5]]
        min_front = min(sensor_frente)

        if min_front < 0.40: # si tenemos un obstaculo a 40cm, giramos
            if not self.obstacle_close:
                self.turn_dir = random.choice([-1.0, 1.0])
            self.obstacle_close = True
        else:
            self.obstacle_close = False

    def control_loop(self):
        now = self.get_clock().now().to_msg()

        cmd = TwistStamped()
        cmd.header.stamp = now
        cmd.header.frame_id = 'base_link'

        if self.obstacle_close:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.6 * self.turn_dir
        else:
            cmd.twist.linear.x = 0.20 # movemos a 20cm/s
            cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.cmd_pub.publish(stop_msg)

def main():
    rclpy.init()
    node = RoomExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
