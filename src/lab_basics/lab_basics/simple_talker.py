import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleTalker(Node):
   def __init__(self):
       super().__init__('simple_talker')
       self.publisher_ = self.create_publisher(String, 'chatter', 10)
       self.timer = self.create_timer(1.0, self.publish_message)


   def publish_message(self):
       msg = String()
       msg.data = 'hello from ROS 2'
       self.publisher_.publish(msg)
       self.get_logger().info(f'Published: {msg.data}')


def main():
   rclpy.init()
   node = SimpleTalker()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
