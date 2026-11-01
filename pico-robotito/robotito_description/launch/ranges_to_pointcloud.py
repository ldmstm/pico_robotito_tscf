import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range, PointCloud2, PointField, LaserScan
from std_msgs.msg import Header
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

# En Robotito solo tenemos 6 ToF fijos que rotan con el robot.
# Nav2 realmente quiere tener un LaserScan o PointCloud, entonces
# transformamos los datos de entrada a esos formatos
class TofBridgeNode(Node):
    def __init__(self):
        super().__init__('tof_bridge_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_ranges = {}
        self.sensor_frames = [f'sensor_{i}_link' for i in range(6)]
        self.detected_max_range = 2.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        for i in range(6):
            topic = f'/sensor_{i}_range'
            self.create_subscription(
                Range,
                topic,
                lambda msg, idx=i: self.range_callback(msg, idx),
                sensor_qos
            )

        self.cloud_pub = self.create_publisher(PointCloud2, '/tof_cloud', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.timer = self.create_timer(1.0 / 15.0, self.publish_aggregated)
        self.get_logger().info('Iniciado puente /sensor_N_range -> /scan')

    def range_callback(self, msg: Range, idx: int):
        self.latest_ranges[idx] = msg
        if msg.max_range > 0.1 and not math.isinf(msg.max_range):
            self.detected_max_range = float(msg.max_range)

    def publish_aggregated(self):
        now = self.get_clock().now().to_msg()
        points_3d = []

        num_bins = 360
        scan_ranges = [float('inf')] * num_bins

        for idx, msg in self.latest_ranges.items():
            r = float(msg.range)
            if math.isnan(r) or math.isinf(r) or r < msg.min_range or r >= msg.max_range:
                continue

            frame_id = self.sensor_frames[idx]

            pt_sensor = PointStamped()
            pt_sensor.header.frame_id = frame_id
            pt_sensor.point.x = r
            pt_sensor.point.y = 0.0
            pt_sensor.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    frame_id,
                    rclpy.time.Time()
                )
                pt_base = do_transform_point(pt_sensor, transform)
                x, y, z = pt_base.point.x, pt_base.point.y, pt_base.point.z
                points_3d.append((x, y, z))

                # las distancias del ToF son un punto fijo pero les aplicamos un
                # arco de +/- 5 grados para simular un lidar
                # estos puntos NO SON REALES, solo el del medio lo es, siguen
                # habiendo puntos ciegos
                planar_dist = math.hypot(x, y)
                angle_rad = math.atan2(y, x) # [-pi, +pi]
                center_bin = int(round(math.degrees(angle_rad - (-math.pi))))

                for offset in range(-5, 6):
                    bin_idx = (center_bin + offset) % 360
                    wall_dist = planar_dist / math.cos(math.radians(offset))
                    if wall_dist < scan_ranges[bin_idx]:
                        scan_ranges[bin_idx] = wall_dist

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(str(e))
                continue

        header = Header(stamp=now, frame_id='base_footprint')
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_data = b''.join([struct.pack('fff', p[0], p[1], p[2]) for p in points_3d])
        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points_3d),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(points_3d),
            data=cloud_data
        )
        self.cloud_pub.publish(cloud_msg)

        scan_msg = LaserScan(
            header=header,
            angle_min=-math.pi,
            angle_max=math.pi - math.radians(1.0),
            angle_increment=math.radians(1.0),
            time_increment=0.0,
            scan_time=1.0 / 15.0,
            range_min=0.02,
            range_max=self.detected_max_range + 0.1,
            ranges=scan_ranges
        )
        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TofBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
