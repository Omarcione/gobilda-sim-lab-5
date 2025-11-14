from enum import Enum
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from sensor_msgs.msg import Odometry, Imu # type: ignore
            
class EKFNode(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('ekf_node')

        lidar_buffer = []
        odom_buffer = []
        
        # Functions running at 10Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

		# lidar odom subscriber
        self.odom_sub = self.create_subscription(Odometry,'/kiss-icp/odom', self.odom_callback, 10)
		
		# imu sub
        self.imu_sub = self.create_subscription(Imu,'/oak/camera/imu_data/', self.imu_callback, 10)

    def ekf_predict(self, imu_buffer, odom_buffer):
        return
    
    # Callback for the events
    
    def odom_callback(self, msg):
        pos_x, pos_y, pos_z = msg.pose.pose.position
        
        self.odom_buffer.append()
        return
	
    def imu_callback(self, msg):
      return
    
    def timer_callback(self):        
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