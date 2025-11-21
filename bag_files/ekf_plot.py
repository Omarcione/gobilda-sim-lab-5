# Reads from ROS2 bag and plots EKF results
import rosbag2_py
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
import math

# Extract yaw from quaternion for LiDAR odom
def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw (theta) from a geometry_msgs/Quaternion."""
    # Standard yaw extraction from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def extract_ekf_data(data: list[(float, float, float, float, list[float])]):
    storage_options = rosbag2_py.StorageOptions(
        uri='/ekf_bag', storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/ekf/odom':
            odom_msg = Odometry()
            odom_msg.deserialize(data)
            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            # Extract yaw from quaternion
            theta = yaw_from_quaternion(odom_msg.pose.pose.orientation)
            covariance = odom_msg.pose.covariance
            data.append((t, x, y, theta, covariance))

def plot_ekf_results(data):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("EKF Position and Covariance")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.axis("equal")

    xs = [d[1] for d in data]
    ys = [d[2] for d in data]
    thetas = [d[3] for d in data]
    covs = [d[4] for d in data]

    # Plot trajectory
    ax.plot(xs, ys, 'b-', label='EKF Path')

    # Plot poses and covariance ellipses
    for (x, y, theta, cov) in zip(xs, ys, thetas, covs):
        # Orientation arrow
        ax.arrow(x, y, 0.2 * math.cos(theta), 0.2 * math.sin(theta),
                 head_width=0.05, head_length=0.1, fc='r', ec='r')

        # 2×2 covariance in x,y
        cov_xy = np.array([[cov[0], cov[1]],
                           [cov[6], cov[7]]])

        # Eigen-decomposition to get ellipse
        w, v = np.linalg.eig(cov_xy)
        angle = math.degrees(math.atan2(v[1, 0], v[0, 0]))
        width, height = 2 * np.sqrt(w)  # 1-sigma ellipse

        ell = Ellipse((x, y), width, height, angle=angle,
                      edgecolor='g', facecolor='none', alpha=0.6)
        ax.add_patch(ell)

    ax.legend()
    # Center the plot around (0, 0)
    ax.set_xlim(min(xs + [0]) - 1, max(xs + [0]) + 1)
    ax.set_ylim(min(ys + [0]) - 1, max(ys + [0]) + 1)
    ax.plot(0, 0, 'ko', label='Origin')

    plt.show()


def main(): 
    data = [] # list of tuples (time, x, y, theta, covariance)

    extract_ekf_data(data)
    plot_ekf_results(data)  

if __name__ == "__main__":
    main()