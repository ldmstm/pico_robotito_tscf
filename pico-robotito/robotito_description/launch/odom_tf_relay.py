import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# A veces la conexion de Wi-Fi es inestable pero tenemos que seguir
# publicando datos a una tasa de frecuencia regular.
# Escuchamos en /odom y mantenemos un bufer publicado regularmente en /odom_synced
class OdomTfRelay(Node):
    def __init__(self):
        super().__init__('odom_tf_relay')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom_synced', 10)
        self.latest_odom = None

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data
        )

        frec = 30.0
        self.create_timer(1.0 / frec, self.broadcast_tf)
        self.get_logger().info(f'Iniciado el relay /odom -> TF ({frec}Hz)')

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        synced_msg = msg
        synced_msg.header.stamp = self.get_clock().now().to_msg()
        synced_msg.header.frame_id = 'odom'
        synced_msg.child_frame_id = 'base_footprint'
        self.odom_pub.publish(synced_msg)

    def broadcast_tf(self):
        if self.latest_odom is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.latest_odom.pose.pose.position.x
        t.transform.translation.y = self.latest_odom.pose.pose.position.y
        t.transform.translation.z = self.latest_odom.pose.pose.position.z
        t.transform.rotation = self.latest_odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
