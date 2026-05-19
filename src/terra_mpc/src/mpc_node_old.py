#!/usr/bin/env python3
import copy
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String, Float32MultiArray
from fpn_msgs.msg import MPCInput, MPCOutput
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseArray, Pose
from sensor_msgs.msg import Image
import yaml
import rospkg
from mpc.mpc import MPC_CONTROLLER
from utils.mpc_utils import euler_from_quaternion, quaternion_from_euler

class MPC_NODE(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')  # Initialize the node

        # Declare parameters with their default values
        self.declare_parameter('alpha', 1)
        self.declare_parameter('use_delay', False)
        self.declare_parameter('eps', 1e-6)
        self.declare_parameter('gain_ctrack_error_x', 10)
        self.declare_parameter('gain_ctrack_error_y', 10)
        self.declare_parameter('gain_ctrack_error_theta', 15)
        self.declare_parameter('terminal_cost_multiplier_x', 15)
        self.declare_parameter('terminal_cost_multiplier_y', 15)
        self.declare_parameter('terminal_cost_multiplier_theta', 50)
        self.declare_parameter('gain_control_effort_linear', 2.5)
        self.declare_parameter('gain_control_effort_angular', 1.0)
        self.declare_parameter('dt', 0.2)
        self.declare_parameter('N', 15)
        self.declare_parameter('v_ref', 0.3)
        self.declare_parameter('Lbase', 0.3)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('v_lin_max', 0.6)
        self.declare_parameter('v_ang_max', 6.0)
        self.declare_parameter('n_states', 3)
        self.declare_parameter('n_controls', 2)
        self.declare_parameter('mu', 1.0)
        self.declare_parameter('nu', 0.2)
        self.declare_parameter('wp_topic_name', "path")
        self.declare_parameter('twist_topic_name', "mpc_cmd_vel")
        self.declare_parameter('verbose', False)

        self.params = {
            'verbose': True,
            'frame_id': "base_footprint",
        }

        # Initialize reference path
        self.mpc_reference = {'x': [], 'y': [], 'theta': [], 'speed': np.array([]), 'x_coeff': np.array([]), 'y_coeff': np.array([])}
        self.odom_data = [0.0, 0.0, 0.0, 0.0]

        # print("It has:",self.get_parameter('gain_ctrack_error_x').get_parameter_value().double_value)
        self.update_params()

        # Create MHE object
        self.mpc = MPC_CONTROLLER(self.params)

        # Parameters
        twist_topic = self.get_parameter('twist_topic_name').get_parameter_value().string_value
        wp_topic = self.get_parameter('wp_topic_name').get_parameter_value().string_value

        # Subscribers
        self.create_subscription(Odometry, "/terrasentia/ekf", self.odom_callback, 1)
        self.create_subscription(Path, wp_topic, self.mpc_callback, 1)
        self.create_subscription(String, '/robot_state', self.robot_stateCb, 1)
        self.create_subscription(Float32MultiArray, '/terrasentia/vision/distance', self.distanceCb, 1)
        self.create_subscription(TwistStamped, '/terrasentia/mpc_node/PID_cmd_vel', self.PID_cmd_velCb, 1)

        # Publisher
        self.pub_twist = self.create_publisher(TwistStamped, twist_topic, 1)
        self.pub_pred_vals = self.create_publisher(PoseArray, "mpc_node/mpc_pred_vals", 1)
        self.pub_pts_car = self.create_publisher(PoseArray, "mpc_node/pts_car", 1)
        self.pub_output = self.create_publisher(MPCOutput, "mpc_node/output", 1)

        # Set run_step to wait for reference
        self.run_step = False
        self.start_time = time.time()
        self.stop_flag = False
        self.robot_state = 'Stopped'
        self.count_rotation = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # ROS Rate at 40Hz
        self.timer = self.create_timer(0.025, self.run_mpc)  # 40Hz

    def myloginfo(self, msg=''):
        self.get_logger().info('[' + str(self.get_name()) + '] ' + str(msg))

    def mylogwarn(self, msg=''):
        self.get_logger().warn('[' + str(self.get_name()) + '] ' + str(msg))

    def mylogerr(self, msg=''):
        self.get_logger().error('[' + str(self.get_name()) + '] ' + str(msg))

    def robot_stateCb(self, msg):
        self.robot_state = msg.data

    def PID_cmd_velCb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z

    def update_params(self):
        # if not self.has_parameter('gain_ctrack_error_x'):
        #     self.get_logger().error('MPC params under name ' + str(name) + ' were not found!!!')
        #     return False
        # self.get_logger().info('Loading MPC params under name ' + str(name) + '!!!')

        # Load parameters
        self.params['alpha'] = self.get_parameter('alpha').get_parameter_value().double_value
        self.params['dt'] = self.get_parameter('dt').get_parameter_value().double_value
        self.params['N'] = self.get_parameter('N').get_parameter_value().integer_value
        self.params['Lbase'] = self.get_parameter('Lbase').get_parameter_value().double_value
        self.params['v_max'] = self.get_parameter('v_max').get_parameter_value().double_value
        self.params['v_lin_max'] = self.get_parameter('v_lin_max').get_parameter_value().double_value
        self.params['v_ang_max'] = self.get_parameter('v_ang_max').get_parameter_value().double_value
        self.params['n_states'] = self.get_parameter('n_states').get_parameter_value().integer_value
        self.params['n_controls'] = self.get_parameter('n_controls').get_parameter_value().integer_value
        self.params['mu'] = self.get_parameter('mu').get_parameter_value().double_value
        self.params['nu'] = self.get_parameter('nu').get_parameter_value().double_value
        self.params['eps'] = self.get_parameter('eps').get_parameter_value().double_value
        self.params['v_ref'] = self.get_parameter('v_ref').get_parameter_value().double_value
        self.params['gain_ctrack_error_x'] = self.get_parameter('gain_ctrack_error_x').get_parameter_value().double_value
        self.params['gain_ctrack_error_y'] = self.get_parameter('gain_ctrack_error_y').get_parameter_value().double_value
        self.params['gain_ctrack_error_theta'] = self.get_parameter('gain_ctrack_error_theta').get_parameter_value().double_value
        self.params['terminal_cost_multiplier_x'] = self.get_parameter('terminal_cost_multiplier_x').get_parameter_value().double_value
        self.params['terminal_cost_multiplier_y'] = self.get_parameter('terminal_cost_multiplier_y').get_parameter_value().double_value
        self.params['terminal_cost_multiplier_theta'] = self.get_parameter('terminal_cost_multiplier_theta').get_parameter_value().double_value
        self.params['gain_control_effort_linear'] = self.get_parameter('gain_control_effort_linear').get_parameter_value().double_value
        self.params['gain_control_effort_angular'] = self.get_parameter('gain_control_effort_angular').get_parameter_value().double_value
        self.params['use_delay'] = self.get_parameter('use_delay').get_parameter_value().bool_value
        self.params['verbose'] = self.get_parameter('verbose').get_parameter_value().bool_value
        return True

    def run_mpc(self):
        # Run only when a new reference path arrives
        if not self.run_step:
            return

        mpc_reference = copy.deepcopy(self.mpc_reference)

        if self.params['verbose']:
            self.get_logger().info("MPC wps are not empty")
        
        start = time.time()
        u, mpc_output, ss_error = self.mpc.solve_mpc(mpc_reference, self.odom_data[3])

        if self.params['verbose']:
            self.get_logger().info(f'solve_mpc time: {time.time() - start}')
            self.get_logger().info(f'u: {u}')
            self.get_logger().info(f'mpc_output: {mpc_output}')
            self.get_logger().info(f'ss_error: {ss_error}')

        # Publish command
        mpc_cmd = TwistStamped()
        mpc_cmd.header.stamp = self.get_clock().now().to_msg()

        if self.robot_state == 'Stopped':
            self.get_logger().info("Stopped")
            mpc_cmd.twist.linear.x = 0.0
            mpc_cmd.twist.angular.z = 0.0

        elif self.robot_state == 'Moving':
            self.get_logger().info("Moving")
            mpc_cmd.twist.linear.x = u[0, 0] 
            mpc_cmd.twist.angular.z = -u[1, 0]
            self.stop_flag = False

        elif self.robot_state == 'Navigation_ended':
            self.get_logger().info("Navigation_ended")
            mpc_cmd.twist.linear.x = 0.0
            mpc_cmd.twist.angular.z = 0.0

        elif self.robot_state == 'Turning':
            self.get_logger().info("Turning")
            self.count_rotation = 0.0
            mpc_cmd.twist.angular.z = self.angular_vel
            mpc_cmd.twist.linear.x = self.linear_vel

        # Publish commands
        self.pub_twist.publish(mpc_cmd)
        self.debugPubs(mpc_output, mpc_reference)

        self.run_step = False

    def distanceCb(self, msg):
        self.dl, self.dr = msg.data
        self.distance_ratio = self.dl / (self.dl + self.dr)

    def odom_callback(self, msg):
        _, _, heading = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        # add new odometry reading to the buffer
        self.odom_data = [msg.pose.pose.position.x, msg.pose.pose.position.y, heading, msg.twist.twist.angular.z]

    def mpc_callback(self, mpc_msg):
        self.run_step = False  # Prevent MPC from running while preparing the reference
        if len(mpc_msg.poses) == 0:
            if self.params['verbose']:
                self.get_logger().info("Received empty path")
            self.mpc_reference['x'].clear()
            self.mpc_reference['y'].clear()
            self.mpc_reference['theta'].clear()
            self.mpc_reference['speed'] = np.array([])

        elif len(mpc_msg.poses) < 2:
            if self.params['verbose']:
                self.get_logger().info("Path arrived with less than 4 points...")
            # Follow the single point without regression
            self.mpc_reference['x'].clear()
            self.mpc_reference['y'].clear()
            self.mpc_reference['theta'].clear()
            self.mpc_reference['speed'] = np.ones(self.params['N'] + 1) * self.params['v_ref']
            for t in range(self.params['N'] + 1):
                x_ref = mpc_msg.poses[0].pose.position.x
                y_ref = mpc_msg.poses[0].pose.position.y
                heading_ref = np.arctan2(y_ref, x_ref)

                # Check if speed is negative and invert the heading to face forward
                if self.mpc_reference['speed'][t] < 0:
                    heading_ref = (heading_ref + np.pi) % (2 * np.pi) - np.pi
                self.mpc_reference['x'].append(x_ref)
                self.mpc_reference['y'].append(y_ref)
                self.mpc_reference['theta'].append(heading_ref)

        else:
            if self.params['verbose']:
                self.get_logger().info(f"Received path size: {len(mpc_msg.poses)}")
                self.get_logger().info(f"\nCalling interpolatePath with ds {self.params['v_ref'] * self.params['dt']}")

            # Number of points to use in the regression algorithm
            regress_length = len(mpc_msg.poses)
            wps_x = [mpc_msg.poses[0].pose.position.x]
            wps_y = [mpc_msg.poses[0].pose.position.y]
            wps_time = [0.0]

            for i in range(1, regress_length):
                wps_x.append(mpc_msg.poses[i].pose.position.x)
                wps_y.append(mpc_msg.poses[i].pose.position.y)
                wps_time.append(wps_time[i - 1] + np.sqrt((wps_x[i] - wps_x[i - 1])**2 + (wps_y[i] - wps_y[i - 1])**2) / np.abs(self.params['v_ref']))

            # Fit 3rd order polynomial
            self.mpc_reference['x_coeff'] = np.polyfit(wps_time, wps_x, 3)
            self.mpc_reference['y_coeff'] = np.polyfit(wps_time, wps_y, 3)
            self.mpc_reference['speed'] = np.ones(self.params['N'] + 1) * self.params['v_ref']

            self.mpc_reference['x'].clear()
            self.mpc_reference['y'].clear()
            self.mpc_reference['theta'].clear()

            for t in range(self.params['N'] + 1):
                x_ref = np.polyval(self.mpc_reference['x_coeff'], t * self.params['dt'])
                y_ref = np.polyval(self.mpc_reference['y_coeff'], t * self.params['dt'])
                dxdt_ref = np.polyval(np.polyder(self.mpc_reference['x_coeff']), t * self.params['dt'])
                dydt_ref = np.polyval(np.polyder(self.mpc_reference['y_coeff']), t * self.params['dt'])
                heading_ref = np.arctan2(dydt_ref, dxdt_ref)

                if self.mpc_reference['speed'][t] < 0:
                    heading_ref = (heading_ref + 2 * np.pi) % (2 * np.pi) - np.pi

                self.mpc_reference['x'].append(x_ref)
                self.mpc_reference['y'].append(y_ref)
                self.mpc_reference['theta'].append(heading_ref)

        # Now MPC can run
        self.run_step = True

    def debugPubs(self, mpc_output, mpc_reference):
        # Publish MPC predicted path
        pa = PoseArray()
        pa.header.frame_id = self.params['frame_id']
        for i in range(mpc_output.shape[1]):
            pose = Pose()
            pose.position.x = mpc_output[0, i]
            pose.position.y = mpc_output[1, i]
            q = quaternion_from_euler(0, 0, mpc_output[2, i])
            pose.orientation.w = q[0]
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]
            pa.poses.append(pose)

        self.pub_pred_vals.publish(pa)

        # Publish MPC predicted output
        mpc_out = MPCOutput()
        for i in range(mpc_output.shape[1]):
            mpc_out.x.append(mpc_output[0, i])
            mpc_out.y.append(mpc_output[1, i])
        self.pub_output.publish(mpc_out)

        # Publish MPC reference path
        ref_pa = PoseArray()
        ref_pa.header.frame_id = self.params['frame_id']
        for i in range(len(mpc_reference['x'])):
            pose = Pose()
            pose.position.x = mpc_reference['x'][i]
            pose.position.y = mpc_reference['y'][i]
            q = quaternion_from_euler(0, 0, mpc_reference['theta'][i])
            pose.orientation.w = q[0]
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]
            ref_pa.poses.append(pose)

        self.pub_pts_car.publish(ref_pa)

# def publish_params_from_yaml(namespace, yaml_file):
#     with open(yaml_file, 'r') as stream:
#         params = yaml.safe_load(stream)

#     for key, value in params.items():
#         full_param_name = f"{namespace}/{key}"
#         rclpy.parameter.set_parameter(full_param_name, value)
#         self.get_logger().info(f"Published parameter '{full_param_name}' with value {value}")

def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPC_NODE()
    rclpy.spin(mpc_node)
    mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
