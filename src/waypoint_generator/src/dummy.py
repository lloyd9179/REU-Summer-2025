import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random
import time

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')

        # Create publishers for heading and distance topics
        self.pub_heading = self.create_publisher(Float32MultiArray, '/terrasentia/vision/heading', 10)
        self.pub_distance = self.create_publisher(Float32MultiArray, '/terrasentia/vision/distance', 10)

        # Publish at a fixed rate
        self.timer = self.create_timer(0.5, self.publish_dummy_data)  # 0.5 seconds interval (2 Hz)

    def publish_dummy_data(self):
        # Generate random heading and distance values
        heading_msg = Float32MultiArray()
        distance_msg = Float32MultiArray()

        # Random heading value (just one element in the array)
        heading_value = random.uniform(-0.5, 0.5)  # Random heading between -0.5 and 0.5 radians
        heading_msg.data = [heading_value]

        # Random distance values (two elements for left and right distances)
        distance_left = random.uniform(0, 2)  # Random distance between 0 and 2 meters
        distance_right = random.uniform(0, 2)  # Random distance between 0 and 2 meters
        distance_msg.data = [distance_left, distance_right]

        # Publish the messages
        self.pub_heading.publish(heading_msg)
        self.pub_distance.publish(distance_msg)

        # Log the published data
        self.get_logger().info(f'Published Heading: {heading_msg.data}')
        self.get_logger().info(f'Published Distance: {distance_msg.data}')

def main(args=None):
    rclpy.init(args=args)

    # Initialize the node
    node = DummyPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the ROS2 system when exiting
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
