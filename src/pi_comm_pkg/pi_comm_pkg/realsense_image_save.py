#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('realsense_image_saver_node')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Get the directory of the current file
        self.base_direc = os.path.dirname(__file__) + '/realsense_pan_tilt_images'

        if not os.path.exists(self.base_direc):
            # Create the directory if it doesn't exist
            os.makedirs(self.base_direc)
            print(f"Directory {self.base_direc} was created.")
        else:
            print(f"Directory {self.base_direc} already exists.")

        self.image_counter = 1
        self.capture_image = True
        self.number_of_poses = 9   #3pan*3tilt, if you want to change this, go to pan_tilt_pub.py
        self.imagesavecb = 0
        items = os.listdir(self.base_direc)
        # Count how many of those items are directories
        self.folder_count = sum(1 for item in items if os.path.isdir(os.path.join(self.base_direc, item)))
        # Subscribe to the flag topic
        self.create_subscription(Bool, '/hunter/img_save_flag_realsense', self.capture_image_callback, 1)

        # Subscribe to the image topic
        self.create_subscription(Image, '/camera/camera/color/image_raw',self.realsense_imgCB, 10)
        # time.sleep(15.0)

    def capture_image_callback(self, msg):
        """Callback function to receive the capture flag."""
        self.get_logger().info("Flag received:")
        # time.sleep(5)
        self.capture_image = msg.data

        if self.capture_image:
            self.get_logger().info("Flag received: Capturing image now.")
            self.imagesavecb = 1
            self.capture_image = False

    def realsense_imgCB(self, msg):
        """Callback function to receive the image and save it if the flag is set."""
        if self.imagesavecb:
            self.imagesavecb = 0
            self.get_logger().info("Processing image...")
            
            # self.image_counter = 1
            self.save_folder = self.base_direc + f'/data_collection_{self.folder_count+1}'
            self.process_image(msg, self.save_folder, self.image_counter)
            self.image_counter += 1
            if self.image_counter/self.number_of_poses>1:
                self.image_counter = 1
                items = os.listdir(self.base_direc)
                # Count how many of those items are directories
                self.folder_count = sum(1 for item in items if os.path.isdir(os.path.join(self.base_direc, item)))
            

    def process_image(self, msg, save_folder, image_counter):
        """Process and save the image."""
        # Convert ROS image message to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # 'bgr8' is common for color images

        # Ensure the save folder exists
        if not os.path.exists(save_folder):
            os.makedirs(save_folder)

        # Construct image save path
        img_save_path = os.path.join(save_folder, f'img_{image_counter}.jpg')
        self.get_logger().info(f"Saving image at {img_save_path}")

        # Save image using OpenCV
        cv2.imwrite(img_save_path, image)

def main(args=None):
    print("hi1")
    rclpy.init(args=args)
    print("hi2")

    image_saver_node = ImageSaverNode()
    print("hi3")

    rclpy.spin(image_saver_node)

    # Shutdown the node
    image_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
