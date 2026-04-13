import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class GreetingService(Node):
   def __init__(self):
       super().__init__('greeting_service')
       self.service = self.create_service(Trigger, 'say_hello', self.callback)
                                         
   def callback(self, request, response):
       response.success = True
       response.message = 'Hello from the service'
       self.get_logger().info('Service was called')
       return response


def main():
   rclpy.init()
   node = GreetingService()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
