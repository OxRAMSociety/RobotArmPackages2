import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import cv2


class CompressedImagePublisher(Node):
    def __init__(self, frequency=30):
        super().__init__("compressed_image_publisher")

        # Default value
        self.declare_parameter("output_topic", "/image_raw/compressed")
        output_topic = self.get_parameter("output_topic").value

        self.declare_parameter("input_path", rclpy.Parameter.Type.STRING)
        input_path = self.get_parameter("input_path").value
        # Loads the image as a NumPy Array
        image = cv2.imread(input_path)
        assert image is not None, "Image could not be read"

        # Processes it into a compressed message format
        self.message = CvBridge().cv2_to_compressed_imgmsg(image, "jpeg")
        self.message.format = "rgb8; jpeg compressed bgr8"

        self.publisher = self.create_publisher(CompressedImage, output_topic, 10)

        self.timer = self.create_timer(1.0 / frequency, self.publish_new)

        self.get_logger().info(f"Publishing {input_path} on topic {output_topic}")

    def publish_new(self):
        self.publisher.publish(self.message)


def main(args=None):
    # Initiate the node
    rclpy.init(args=args)
    publisher_node = CompressedImagePublisher()

    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
