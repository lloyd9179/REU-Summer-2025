#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import numpy as np
from scipy.optimize import minimize, NonlinearConstraint
import matplotlib.pyplot as plt
import time
class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Vehicle parameters
        self.robot_dims = (0.55, 0.5, 0.6)  # Length, width, height
        self.L = self.robot_dims[0]
        self.interpolation_pts = 3 #number of interpolation points between two original wps = self.interpolation_pts-1
        self.N = 10  # Prediction horizon, this cant be more then len(wps_original)*self.interpolation_pts
        self.dt = 0.6 # Time step
        self.omega_max = 2  # Max angular velocity
        self.v_max = 0.5  # Max linear velocity
        self.v_min = 0.0  # Min linear velocity
        self.relative_waypoints = None
        self.horizon_length = 5
        self.ang_vel_cost_weight = 1.5
        self.cte_weight = 1
        self.robot_state = None

        # Current state variables
        self.current_velocity = 0.0
        self.current_steering_angle = 0.0

        self.steering_min = -np.pi / 4  # Min steering angle
        self.steering_max = np.pi / 4   # Max steering angle

        # Subscribers
        self.create_subscription(Path, '/hunter/path', self.path_callback, 1)
        self.create_subscription(Odometry, '/odom', self.velocity_callback, 1)
        self.create_subscription(Float32, '/current_steering_angle', self.steering_angle_callback, 1)
        self.create_subscription(String,'/hunter/robot_state',self.robot_stateCB,10)

        # Publishers
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.steering_pub = self.create_publisher(Float32, '/computed_steering', 1)


    def robot_stateCB(self,msg):
        self.robot_state = msg.data

    def path_callback(self, msg):
        # Extract waypoints from the Path message
        print("hohohoho")
        waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            waypoints.append((x, y))

        if self.N > (len(waypoints)-1)*(self.interpolation_pts)+1:
            self.N = (len(waypoints)-1)*(self.interpolation_pts) +1
            print("Number of horizon points N, exceeds the number of waypoints generated, so it is clipped to max wps.")
        
        self.relative_waypoints = waypoints
        # self.relative_waypoints = self.expand_waypoints(waypoints,self.interpolation_pts)

        if self.relative_waypoints is not None:
            start = time.time()
            self.solve_mpc()
            end = time.time()


    def expand_waypoints(self,waypoints, N):
        """
        Expands a set of waypoints into N equidistant points between each pair of consecutive waypoints.
        
        Args:
            waypoints (list of tuples): List of (x, y) tuples representing waypoints.
            N (int): Number of equidistant points to generate between each pair of consecutive waypoints.
            
        Returns:
            expanded_points (list of tuples): List of (x, y) tuples with interpolated equidistant points.
        """
        expanded_points = []

        for i in range(len(waypoints) - 1):
            # Start and end points for each segment
            start = np.array(waypoints[i])
            end = np.array(waypoints[i + 1])
            
            # Generate N-1 points between start and end
            for j in range(N):
                t = j / N  # interpolation factor
                point = (1 - t) * start + t * end
                expanded_points.append(tuple(point))

        expanded_points.append(waypoints[-1])
        
        return expanded_points
    

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

        ang_vel_change_cost = abs(self.ang_vel_cost(u))
        # print("Costs:", cost, ang_vel_change_cost)
        
        cost = self.cte_weight*cost + self.ang_vel_cost_weight*ang_vel_change_cost
        return cost

    def steering_constraint(self, u):
        v = u[:self.N]
        omega = u[self.N:]
        steering_angles = np.arctan((omega * self.L) / np.maximum(v, 1e-5))
        return steering_angles
    
    def ang_vel_cost(self,u):
        # steering_angles = self.steering_constraint(u)
        ang_vels = u[self.N:]
        ang_vels_increment_sum = 0
        for i in range(len(ang_vels)-1):
            ang_vels_increment_sum += abs(ang_vels[i+1]-ang_vels[i])

        return ang_vels_increment_sum

    def solve_mpc(self):
        x0, y0, theta0 = 0, 0, 0  # Assume starting from origin
        u0 = np.hstack((np.full(self.N, self.current_velocity), np.full(self.N, self.current_ang_vel)))

        bounds = [(self.v_min, self.v_max)] * self.N + [(-self.omega_max, self.omega_max)] * self.N


        steering_constraint_obj = NonlinearConstraint(self.steering_constraint, self.steering_min, self.steering_max)

        # self.relative_waypoints = self.expand_waypoints(self.relative_waypoints,self.interpolation_pts)
        # result = minimize(
        #     self.objective, u0, args=(x0, y0, theta0, self.relative_waypoints),
        #     bounds=bounds, method='SLSQP', constraints=[steering_constraint_obj]
        # )
        result = minimize(
            self.objective, u0, args=(x0, y0, theta0, self.relative_waypoints),
            bounds=bounds, method='SLSQP'
        )
        optimal_v = result.x[:self.N]
        optimal_omega = result.x[self.N:]
        optimal_steering = np.arctan((optimal_omega * self.L) / np.maximum(optimal_v, 1e-5))

        # Publish the first optimal velocity and steering
        self.publish_controls(optimal_v[0], optimal_omega[0], optimal_steering[0])


    def visualize(self, v, omega, waypoints, offset_distance):
        # Visualize the predicted trajectory compared to the waypoints
        plt.figure(figsize=(10, 6))

        # Starting state
        x, y, theta = 0, 0, 0
        trajectory_x = [x]
        trajectory_y = [y]

        # Apply dynamics and simulate the trajectory
        for t in range(self.N):
            x, y, theta = self.vehicle_dynamics(x, y, theta, v[t], omega[t])
            trajectory_x.append(x)
            trajectory_y.append(y)

        # Plot the predicted trajectory
        plt.plot(trajectory_x, trajectory_y, 'bo-', label='MPC Predicted Trajectory')

        # Convert waypoints to numpy array
        waypoints = np.array(waypoints)
        plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoints')

        # Calculate unit vector perpendicular to the waypoints line
        deltas = np.diff(waypoints, axis=0)  # Differences between waypoints
        mean_delta = np.mean(deltas, axis=0)  # Average direction of the waypoints line
        perpendicular_vector = np.array([-mean_delta[1], mean_delta[0]])  # Rotate by 90 degrees
        unit_perpendicular = perpendicular_vector / np.linalg.norm(perpendicular_vector)  # Normalize

        # Offset waypoints to create parallel lines
        line1 = waypoints + offset_distance * unit_perpendicular  # Offset upwards
        line2 = waypoints - offset_distance * unit_perpendicular  # Offset downwards

        # Plot the parallel lines
        plt.plot(line1[:, 0], line1[:, 1], 'g--', label='Parallel Line 1')
        plt.plot(line2[:, 0], line2[:, 1], 'm--', label='Parallel Line 2')

        # Add labels and legend
        plt.title('MPC Control: Vehicle Trajectory vs Waypoints and Parallel Lines')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)
        # plt.savefig("/home/indro/tuskegee_robot_ws_v4/src/terra_mpc/src/path.jpg")
        
        # plt.show()


    def publish_controls(self, velocity, ang_vel, steering_angle):
        if self.robot_state == 'moving':
            vel_msg = Twist()
            vel_msg.linear.x = velocity
            vel_msg.angular.z = ang_vel
            self.velocity_pub.publish(vel_msg)

            steering_msg = Float32()
            steering_msg.data = steering_angle
            self.steering_pub.publish(steering_msg)

        if self.robot_state == 'data_collection':
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_pub.publish(vel_msg)

            steering_msg = Float32()
            steering_msg.data = 0.0
            self.steering_pub.publish(steering_msg)
        
        # self.get_logger().info(f'Published velocity: {velocity}, steering angle: {steering_angle}')


def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MPCController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
