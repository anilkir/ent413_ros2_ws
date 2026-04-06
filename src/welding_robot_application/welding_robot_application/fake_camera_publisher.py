from dataclasses import dataclass

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class CameraStream:
    name: str
    frame_id: str
    image_topic: str
    camera_info_topic: str


class FakeCameraPublisher(Node):
    """Publish a synthetic overhead camera stream to demonstrate ROS sensor data flow."""

    def __init__(self):
        super().__init__("fake_camera_publisher")

        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("publish_rate", 5.0)

        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.streams = [
            CameraStream(
                name="overhead_camera",
                frame_id="overhead_camera_optical_frame",
                image_topic="/overhead_camera/image_raw",
                camera_info_topic="/overhead_camera/camera_info",
            ),
        ]

        self.image_publishers = {
            stream.name: self.create_publisher(Image, stream.image_topic, 10)
            for stream in self.streams
        }
        self.info_publishers = {
            stream.name: self.create_publisher(CameraInfo, stream.camera_info_topic, 10)
            for stream in self.streams
        }
        self.fov_marker_publisher = self.create_publisher(
            Marker, "/overhead_camera/fov_marker", 10
        )

        self.tick = 0
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_images)

        for stream in self.streams:
            self.get_logger().info(
                f"Publishing {stream.image_topic} and {stream.camera_info_topic} "
                f"in frame {stream.frame_id}"
            )

    def publish_images(self):
        stamp = self.get_clock().now().to_msg()
        for stream in self.streams:
            image = self.make_image(stream, stamp)
            camera_info = self.make_camera_info(stream, stamp)
            self.image_publishers[stream.name].publish(image)
            self.info_publishers[stream.name].publish(camera_info)
            self.fov_marker_publisher.publish(self.make_fov_marker(stream, stamp))
        self.tick += 1

    def make_image(self, stream: CameraStream, stamp) -> Image:
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = stream.frame_id
        image.height = self.height
        image.width = self.width
        image.encoding = "rgb8"
        image.is_bigendian = False
        image.step = self.width * 3
        image.data = self.make_pattern()
        return image

    def make_camera_info(self, stream: CameraStream, stamp) -> CameraInfo:
        fx = float(self.width)
        fy = float(self.width)
        cx = (self.width - 1.0) / 2.0
        cy = (self.height - 1.0) / 2.0

        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = stream.frame_id
        camera_info.width = self.width
        camera_info.height = self.height
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return camera_info

    def make_fov_marker(self, stream: CameraStream, stamp) -> Marker:
        depth = 0.75
        half_width = depth * 0.45
        half_height = depth * 0.34
        origin = Point(x=0.0, y=0.0, z=0.0)
        top_left = Point(x=-half_width, y=-half_height, z=depth)
        top_right = Point(x=half_width, y=-half_height, z=depth)
        bottom_right = Point(x=half_width, y=half_height, z=depth)
        bottom_left = Point(x=-half_width, y=half_height, z=depth)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = stream.frame_id
        marker.ns = "overhead_camera"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 1.0
        marker.points = [
            origin,
            top_left,
            origin,
            top_right,
            origin,
            bottom_right,
            origin,
            bottom_left,
            top_left,
            top_right,
            top_right,
            bottom_right,
            bottom_right,
            bottom_left,
            bottom_left,
            top_left,
        ]
        return marker

    def make_pattern(self) -> bytearray:
        data = bytearray(self.width * self.height * 3)
        square_size = max(16, min(self.width, self.height) // 8)
        moving_x = (self.tick * 9) % max(1, self.width - square_size)
        moving_y = (self.tick * 5) % max(1, self.height - square_size)

        for y in range(self.height):
            for x in range(self.width):
                index = (y * self.width + x) * 3
                red = 35
                green = (x + y + self.tick * 3) % 256
                blue = (2 * x + self.tick * 2) % 256

                if abs(x - self.width // 2) <= 1 or abs(y - self.height // 2) <= 1:
                    red, green, blue = 255, 255, 255

                if moving_x <= x < moving_x + square_size and moving_y <= y < moving_y + square_size:
                    red, green, blue = 255, 230, 30

                data[index] = red
                data[index + 1] = green
                data[index + 2] = blue

        return data


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
