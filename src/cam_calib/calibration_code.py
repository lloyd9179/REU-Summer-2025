import rclpy
from rclpy.node import Node
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebcamCaptureNode(Node):
    def __init__(self):
        super().__init__('webcam_capture_node')
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Publisher for the Image topic (if you want to publish captured images)
        self.publisher = self.create_publisher(Image, 'webcam_image', 10)
        
        # Timer for saving images at intervals
        self.timer = self.create_timer(3.0, self.capture_and_save_image)  # 3 seconds interval
        
        # Open the webcam (use /dev/video0)
        self.cap = cv2.VideoCapture(2)  # 0 corresponds to /dev/video0
        # Set the desired resolution
        self.width = 640
        self.height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
            return
        
        self.image_count = 0
        self.get_logger().info("Webcam capture started. Saving images every 2 seconds.")
    
    def capture_and_save_image(self):
        # Capture a frame from the webcam
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return
        
        # Resize the image to 240x320
        resized_frame = cv2.resize(frame, (320, 240))  # Resize to 320x240 (width x height)

        # Save the resized image with a timestamp
        timestamp = time.time()
        filepath = '/home/indro/tuskegee_robot_ws_v2/src/cam_calib/calib_images_v2'
        filename = filepath+f"/captured_image_{self.image_count}.jpg"
        cv2.imwrite(filename, resized_frame)
        self.get_logger().info(f"Image saved: {filename}")
        
        # Optionally, publish the resized image to a ROS2 topic (if you want subscribers to get the image)
        ros_image = self.bridge.cv2_to_imgmsg(resized_frame, encoding="bgr8")
        self.publisher.publish(ros_image)
        
        # Display the live image in an OpenCV window
        cv2.imshow('Live Webcam Feed', resized_frame)
        
        # Wait for key press to close the window (useful for live view)
        cv2.waitKey(1)  # OpenCV's window handling
        # Increment the image counter
        self.image_count += 1

    def __del__(self):
        # Release the webcam capture when done
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamCaptureNode()
    
    # Spin the node to keep it running
    rclpy.spin(node)
    
    # Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
