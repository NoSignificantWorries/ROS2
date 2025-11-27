import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2


class DepthListen(Node):
    def __init__(self):
        super().__init__('depth_listen_node')

        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscriber = self.create_subscription(PointCloud2, '/robot/points', self.pointcloud_callback, 10)
        self.scan = None
        self.timer = self.create_timer(0.1, self.move)

        self.safe_distance = 0.8        # порог расстояния для остановки (метры)
        self.max_speed = 0.2            # скорость движения вперед (м/с)

    def pointcloud_callback(self, msg):
        self.scan = msg

    def move(self):
        twist = Twist()

        if self.scan is None:
            twist.linear.x = self.max_speed
            self.publisher.publish(twist)
            self.get_logger().info('No data received yet, moving forward')
            return

        # Читаем центр облака точек
        index = (self.scan.width * self.scan.height) // 2 + (self.scan.width // 2)
        points = list(pc2.read_points(self.scan, field_names=("x", "y", "z"), skip_nans=True))

        if len(points) <= index:
            twist.linear.x = self.max_speed
            self.get_logger().info('Central point not available, moving forward')
            self.publisher.publish(twist)
            return

        center_point = points[index]
        dist = center_point[0]  # расстояние по оси X

        tolerance = 0.05  # дельта для зоны "устойчивости"

        if dist > (self.safe_distance + tolerance):
            # Двигаемся вперед
            twist.linear.x = self.max_speed
            self.get_logger().info(f'Path clear (distance {dist:.2f} m), moving forward')
        elif dist > 0 and dist < (self.safe_distance - tolerance):
            # Припятствие близко - отъехать
            twist.linear.x = -0.05
            self.get_logger().info(f'Obstacle close ({dist:.2f} m), moving backward')
        else:
            # Остановка
            twist.linear.x = 0.0
            self.get_logger().info(f'Distance invalid, stopping')

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DepthListen()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
