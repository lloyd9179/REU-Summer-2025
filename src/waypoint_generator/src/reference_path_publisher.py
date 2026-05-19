#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node

class PathPublisher(Node):
    def __init__(self,num_points = 10):
        super().__init__('path_publisher')
        self.deviation = 0.0
        self.angle = 0.0
        self.num_points = num_points

        self.pub = self.create_publisher(Path,'/hunter/path',10)

        self.sub_heading = self.create_subscription(Float32MultiArray,'/hunter/vision/heading',self.update_angle,1)
        self.sub_distance = self.create_subscription(Float32MultiArray,'/hunter/vision/distance',self.update_deviation,1)

    def update_angle(self,msg):
        self.angle = -msg.data[0]
        path = Path()
        path.header.frame_id = 'base_footprint'

        for i in range(self.num_points):
            pose = PoseStamped()
            pose.pose.position.x = float(i)
            pose.pose.position.y = self.deviation

            x_rotated = pose.pose.position.x * np.cos(self.angle) - pose.pose.position.y * np.sin(self.angle)
            y_rotated = pose.pose.position.x * np.sin(self.angle) + pose.pose.position.y * np.cos(self.angle)

            pose.pose.position.x = x_rotated
            pose.pose.position.y = y_rotated 

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)
        print("Published path")
        self.pub.publish(path)

    def update_deviation(self, msg):
        dl = msg.data[0]; dr = msg.data[1]
        self.deviation = (dl-dr)/2      

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the node
    node = PathPublisher()

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