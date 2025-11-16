from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Odometry, Imu # type: ignore
            
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
        return
    
    def publish_state(self):
        """
        Publish fused state as an Odometry message.
        
        You can use the ros2bag CLI to record the state of the robot after it is published. 
        Then you can write a python script to load the data and plot it.
        """
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