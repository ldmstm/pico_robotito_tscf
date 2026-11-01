import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class OmniOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('omni_odom_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.5
        self.wheel_angles = [0.0, 0.0, 0.0]

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('Puente /odom -> base_footprint iniciado')

    def cmd_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def update(self):
        now = self.get_clock().now().to_msg()

        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * self.dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * self.dt
        self.yaw += self.wz * self.dt

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        r = 0.0175  # wheel_radius
        L = 0.06    # wheel_mount_r
        azimuths = [math.pi / 2, -5 * math.pi / 6, -math.pi / 6]
        for i, az in enumerate(azimuths):
            v_wheel = -math.sin(az) * self.vx + math.cos(az) * self.vy + L * self.wz
            self.wheel_angles[i] += (v_wheel / r) * self.dt

        js = JointState()
        js.header.stamp = now
        js.name = ['wheel_0_joint', 'wheel_1_joint', 'wheel_2_joint']
        js.position = self.wheel_angles
        self.joint_pub.sendTransform if False else self.joint_pub.publish(js)

def main():
    rclpy.init()
    node = OmniOdomBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
