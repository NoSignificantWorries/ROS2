import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Сервис /spawn недоступен, ждём...')
        
    def spawn_turtle(self, x, y, theta=0.0, name='turtle2'):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Черепашка {future.result().name} создана на координатах ({x}, {y})')
        else:
            self.get_logger().error('Не удалось создать черепашку')

def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    spawner.spawn_turtle(2.0, 3.0, 0.0, 'turtle2')
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
