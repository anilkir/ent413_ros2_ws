import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleListener(Node):
   def __init__(self):
       super().__init__('simple_listener')
       self.subscription = self.create_subscription(
           String,
           'chatter',
           self.callback,
           10
       )


   def callback(self, msg):
       self.get_logger().info(f'Received: {msg.data}')


def main():
   rclpy.init()
   node = SimpleListener()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
