#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from fpn_msgs.srv import BoolSrv

class state_machine(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.robot_state_pub = self.create_publisher(String,'/hunter/robot_state',10)
        self.arm_state_sub = self.create_subscription(String,'/hunter/data_collection_state',self.arm_stateCB,1)
        self.pan_tilt_sub = self.create_subscription(String,'/hunter/pi_comm_state',self.pi_comm_stateCB, 1)
        self.arm_state_client = self.create_client(BoolSrv, '/kiyo_data_collection_service')
        self.pi_state_client = self.create_client(BoolSrv, '/realsense_data_collection_service') 

        self.start_time = time.time()
        self.stop_freq = 15
        self.stop_flag = False
        self.arm_state = 'stagnant'
        self.pi_comm_state = 'stagnant'
        self.robot_state = 'moving'

        time.sleep(15.0)
        self.state_pub()


    def state_pub(self):
        while True:
            self.current_time = time.time()

            if self.robot_state == 'moving' and round(self.current_time - self.start_time) % self.stop_freq == 0 and (self.current_time - self.start_time) > 1:
                self.arm_state = 'capturing'
                self.pi_comm_state = 'capturing'
                self.data_collection_state()
                self.robot_state = 'moving'
                self.start_time = time.time()
                self.current_time = self.start_time

            if self.robot_state == 'moving':
                self.navigation_state() 

    def navigation_state(self):
        robot_state = 'moving'
        msg = String()
        msg.data = robot_state
        self.robot_state_pub.publish(msg)
        self.robot_state = robot_state
         
    def data_collection_state(self):
        self.get_logger().info("Entered data collection state................")
        robot_state = 'data_collection'
        msg = String()
        msg.data = robot_state
        self.robot_state_pub.publish(msg)
        time.sleep(1.0)
        self.robot_state = robot_state

        self.send_robot_state_request()

        self.get_logger().info("Exiting data collection state into navigation state................")



    def send_robot_state_request(self):
      

        # Prepare the request (adjust according to your actual request structure)
        request = BoolSrv.Request()
        request.boolean = True  # Example request content; modify as needed

        # # Send a service request and wait for the response
        # while not self.arm_state_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().error('Service arm not available, waiting...')
        

        # future_arm = self.arm_state_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future_arm)


        # if future_arm.result():
        #     self.get_logger().info(f"Service call arm succeeded: {future_arm.result().message}")
        # else:
        #     self.get_logger().error(f"Service call arm failed: {future_arm.exception()}")


        # while not self.pi_state_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().error('Service pi not available, waiting...')

        # future_pi = self.pi_state_client.call_async(request)
        # rclpy.spin_until_future_complete(self,future_pi)
        # if future_pi.result():
        #     self.get_logger().info(f"Service call pi succeeded: {future_pi.result().message}")
        # else:
        #     self.get_logger().error(f"Service call pi failed: {future_pi.exception()}")




        #Both arm and pi moving at same time, uncomment above lines if you want arm and pi to move one after another
        # Send a service request and wait for the response
        while not self.arm_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service arm not available, waiting...')
        

        while not self.pi_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service pi not available, waiting...')

        future_arm = self.arm_state_client.call_async(request)
        future_pi = self.pi_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future_arm)


        if future_arm.result():
            self.get_logger().info(f"Service call arm succeeded: {future_arm.result().message}")
        else:
            self.get_logger().error(f"Service call arm failed: {future_arm.exception()}")

        
        # rclpy.spin_until_future_complete(self,future_pi)
        if future_pi.result():
            self.get_logger().info(f"Service call pi succeeded: {future_pi.result().message}")
        else:
            self.get_logger().error(f"Service call pi failed: {future_pi.exception()}")



    # def nav_end_state(self):
    #     robot_state = 'nav_end'
    #     msg = String()
    #     msg.data = robot_state
    #     self.robot_state_pub.publish(msg)

    def arm_stateCB(self, msg):
        self.arm_state = msg.data
         
    def pi_comm_stateCB(self,msg):
        self.pi_comm_state = msg.data

def main(args=None):
	rclpy.init(args = args)
	state_machine_node = state_machine()
	rclpy.spin(state_machine_node)
	state_machine_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
