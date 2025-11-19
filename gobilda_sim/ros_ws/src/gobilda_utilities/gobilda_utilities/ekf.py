from collections import deque
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Odometry, Imu # type: ignore
from geometry_msgs.msg import Quaternion   # for output odom orientation

            
def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

# Extract yaw from quaternion for LiDAR odom
def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw (theta) from a geometry_msgs/Quaternion."""
    # Standard yaw extraction from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# Use this to create a Quaternion from yaw for publishing
def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a Quaternion from yaw (theta) only (no roll/pitch)."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class EKFNode(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('ekf_node')

        # State variables
        self.state_vector = np.zeros((3, 1))  # [x, y, theta]
        self.covariance_matrix = np.eye(3) * 1e-4  # 3x3 Identity matrix as initial covariance
        
            # State vector:
            # [[x],
            # [y],
            # [theta]]

            # Initial covariance matrix:
            # [[0.0001, 0.    , 0.    ],
            # [0.    , 0.0001, 0.    ],
            # [0.    , 0.    , 0.0001]]

        # Process noise (Q)
        # Larger value for theta since IMU drift mainly affects orientation
        self.Q = np.diag([1e-4, 1e-4, 1e-3])

        # Measurement noise (R) – LiDAR odom uncertainty 
        # ChatGPT suggested these based on typical LiDAR odometry performance so may need tuning
        self.R = np.diag([0.01, 0.01, 0.02])

        # Observation model (H): LiDAR directly measures [x, y, theta]
        self.H = np.eye(3)

        # Buffers for incoming data
        self.imu_queue = deque()    # Predict step
        self.odom_queue = deque()   # Update step

        # Keep track of last IMU time to compute dt
        self.last_imu_time = None
        
        # Functions running at 20Hz
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

		# lidar odom subscriber
        self.odom_sub = self.create_subscription(Odometry,'/kiss-icp/odom', self.odom_callback, 10)
		
		# imu sub
        self.imu_sub = self.create_subscription(Imu,'/oak/camera/imu_data/', self.imu_callback, 10)

        # odom pub
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

    def ekf_step(self):
        # Predict using last IMU
        if self.imu_queue:
            imu_msg = self.imu_queue.pop()
            self.predict(imu_msg)

        # Update if LiDAR available
        if self.odom_queue:
            lidar_msg = self.lidar_queue.pop()
            self.update(lidar_msg)

        # Publish fused state
        self.publish_state()
    
    def predict(self, imu_msg):
        """
        Predict step using IMU angular velocity (yaw rate).
        Only theta is predicted; x,y remain unchanged and will be corrected by LiDAR.
        """
        ## Calculate change in time (dt)
        current_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return
        
        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        if dt <= 0:
            return  # No time has passed, skip prediction
        

        ## Predict theta from angular velocity
        angular_velocity_z = imu_msg.angular_velocity.z
        theta = self.state_vector[2, 0]

        theta_prediction = theta + angular_velocity_z * dt
        self.state_vector[2, 0] = theta_prediction


        ## Covariance
        # Motion model Jacobian F. Since we only predict theta, F is just identity.
        F = np.eye(3)

        # Scale Q by dt (simple model: more time, more uncertainty)
        Q_dt = self.Q * dt

        # Update covariance
        # @ == matrix multiplication
        # equation: P' = FPF^T + Q
        self.covariance_matrix = F @ self.covariance_matrix @ F.T + Q_dt
        return
    
    def update(self, lidar_msg):
        """
        Update step using LiDAR odometry:
        Measurement z = [x_lidar, y_lidar, theta_lidar]^T
        """
        # --- Build measurement vector z ---
        x_measurement = lidar_msg.pose.pose.position.x
        y_measurement = lidar_msg.pose.pose.position.y
        theta_measurement = yaw_from_quaternion(lidar_msg.pose.pose.orientation)

        z = np.array([[x_measurement],
                      [y_measurement],
                      [theta_measurement]])

        # --- Innovation y = z - h(x) ---
        # h(x) = x for this model, since LiDAR directly measures state
        y = z - self.state_vector
        # Wrap the angle residual incase new theta crosses the -pi to pi boundary
        y[2, 0] = wrap_angle(y[2, 0])

        # --- Innovation covariance S ---
        P = self.covariance_matrix
        H = self.H
        R = self.R

        S = H @ P @ H.T + R

        # --- Kalman gain ---
        K = P @ H.T @ np.linalg.inv(S)

        # --- State update ---
        self.state_vector = self.state_vector + K @ y

        # Wrap theta to [-pi, pi] after update
        self.state_vector[2, 0] = wrap_angle(self.state_vector[2, 0])

        # --- Covariance update ---
        I = np.eye(3)
        self.covariance_matrix = (I - K @ H) @ P
        return
    
    def publish_state(self):
        """
        Publish fused state as an Odometry message.
        
        You can use the ros2bag CLI to record the state of the robot after it is published. 
        Then you can write a python script to load the data and plot it.
        """
        # Create Odom msg
        odom_msg = Odometry()
        # Fill in header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        # Fill in position
        odom_msg.pose.pose.position.x = self.state_vector[0, 0]
        odom_msg.pose.pose.position.y = self.state_vector[1, 0]
        odom_msg.pose.pose.position.z = 0.0  # No z element
        # Fill in orientation
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.state_vector[2, 0])
        # Fill in covariance
        odom_msg.pose.covariance = self.covariance_matrix.flatten().tolist() + [0]*27  # Fill rest with zeros to make 6x6
        # Publish
        self.odom_pub.publish(odom_msg)
        return
    
    # Callbacks for the events
    def imu_callback(self, msg):
        self.imu_buffer.append(msg)
        return
    
    def odom_callback(self, msg):
        self.odom_buffer.append(msg)
        return
    
    def timer_callback(self): 
        self.ekf_step()       
        return


def main(args=None):
    rclpy.init(args=args)

    ekf_node = EKFNode()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(ekf_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()