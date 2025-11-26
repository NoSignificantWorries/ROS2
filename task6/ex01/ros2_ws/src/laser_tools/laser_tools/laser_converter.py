import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        self.projector = LaserProjection()
        self.sub = self.create_subscription(
            LaserScan, 
            '/robot/scan', 
            self.scan_callback, 
            10
        )
        self.pub = self.create_publisher(
            PointCloud2, 
            '/robot/points', 
            10
        )
        self.get_logger().info('LaserScan to PointCloud2 converter started')
    
    def scan_callback(self, msg):
        try:
            cloud = self.projector.projectLaser(msg)
            self.pub.publish(cloud)
        except Exception as e:
            self.get_logger().error(f'Error converting scan: {str(e)}')

def main():
    rclpy.init()
    node = LaserToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
