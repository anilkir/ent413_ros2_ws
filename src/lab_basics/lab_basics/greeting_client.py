import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class GreetingClient(Node):
   def __init__(self):
       super().__init__('greeting_client')
       self.client = self.create_client(Trigger, 'say_hello')


       while not self.client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('Waiting for service...')


       self.send_request()


   def send_request(self):
       request = Trigger.Request()
       future = self.client.call_async(request)
       rclpy.spin_until_future_complete(self, future)


       response = future.result()
       if response is not None:
           self.get_logger().info(f'Response: {response.message}')
       else:
           self.get_logger().error('Service call failed')


def main():
   rclpy.init()
   node = GreetingClient()
   node.destroy_node()
   rclpy.shutdown()
