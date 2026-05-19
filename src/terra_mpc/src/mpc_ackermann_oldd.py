#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import numpy as np
from scipy.optimize import minimize, NonlinearConstraint
import matplotlib.pyplot as plt

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Vehicle parameters
        self.robot_dims = (0.55, 0.5, 0.6)  # Length, width, height
        self.L = self.robot_dims[0]
        self.N = 10  # Prediction horizon
        self.dt = 3  # Time step
        self.omega_max = np.pi / 4  # Max angular velocity
        self.v_max = 0.5  # Max linear velocity
        self.v_min = 0  # Min linear velocity
        self.relative_waypoints = None

        # Current state variables
        self.current_velocity = 0.0
        self.current_steering_angle = 0.0

        self.steering_min = -np.pi / 6  # Min steering angle
        self.steering_max = np.pi / 6   # Max steering angle

        # Subscribers
        self.create_subscription(Path, '/terrasentia/path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.velocity_callback, 10)
        self.create_subscription(Float32, '/current_steering_angle', self.steering_angle_callback, 10)

        # Publishers
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steering_pub = self.create_publisher(Float32, '/computed_steering', 10)

    def path_callback(self, msg):
        # Extract waypoints from the Path message
        waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            waypoints.append([x, y])
        
        self.relative_waypoints = np.array(waypoints)
        if self.relative_waypoints is not None:
            # print("Starting MPCCCCCCCC")
            self.solve_mpc()

    def velocity_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x
        self.current_ang_vel = msg.twist.twist.angular.z

    def steering_angle_callback(self, msg):
        self.current_steering_angle = msg.data

    def vehicle_dynamics(self, x, y, theta, v, omega):
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + omega * self.dt
        return x_next, y_next, theta_next

    def objective(self, u, x0, y0, theta0, waypoints):
        v = u[:self.N]
        omega = u[self.N:]
        x, y, theta = x0, y0, theta0
        cost = 0

        for t in range(self.N):
            x, y, theta = self.vehicle_dynamics(x, y, theta, v[t], omega[t])
            waypoint_x, waypoint_y = waypoints[min(t, len(waypoints) - 1)]  # Prevent index out of range
            cost += (x - waypoint_x)**2 + (y - waypoint_y)**2

        return cost

    def steering_constraint(self, u):
        v = u[:self.N]
        omega = u[self.N:]
        steering_angles = np.arctan((omega * self.L) / np.maximum(v, 1e-5))
        return steering_angles

    def solve_mpc(self):
        x0, y0, theta0 = 0, 0, 0  # Assume starting from origin
        u0 = np.hstack((np.full(self.N, self.current_velocity), np.full(self.N, self.current_ang_vel)))

        bounds = [(self.v_min, self.v_max)] * self.N + [(-self.omega_max, self.omega_max)] * self.N


        steering_constraint_obj = NonlinearConstraint(self.steering_constraint, self.steering_min, self.steering_max)

        result = minimize(
            self.objective, u0, args=(x0, y0, theta0, self.relative_waypoints),
            bounds=bounds, method='SLSQP', constraints=[steering_constraint_obj]
        )

        optimal_v = result.x[:self.N]
        optimal_omega = result.x[self.N:]
        optimal_steering = np.arctan((optimal_omega * self.L) / np.maximum(optimal_v, 1e-5))

        # Publish the first optimal velocity and steering
        self.publish_controls(optimal_v[0], optimal_omega[0], optimal_steering[0])

    def publish_controls(self, velocity, ang_vel, steering_angle):
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        vel_msg.angular.z = ang_vel
        self.velocity_pub.publish(vel_msg)

        steering_msg = Float32()
        steering_msg.data = steering_angle
        self.steering_pub.publish(steering_msg)

        self.get_logger().info(f'Published velocity: {velocity}, steering angle: {steering_angle}')


def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MPCController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
