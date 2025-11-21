# Reads from ROS2 bag and plots EKF results
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import math

# Extract yaw from quaternion
def yaw_from_quaternion(q) -> float:
    """Extract yaw (theta) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def extract_ekf_data(results):
    # Use correct absolute or relative path
    storage_options = rosbag2_py.StorageOptions(
        uri='./ekf_bag',   # change if needed
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Read all messages
    while reader.has_next():
        topic, raw_data, t = reader.read_next()
        if topic == '/odometry/filtered':  # update if your bag topic differs
            odom_msg = deserialize_message(raw_data, Odometry)
            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            theta = yaw_from_quaternion(odom_msg.pose.pose.orientation)
            covariance = odom_msg.pose.covariance
            results.append((t, x, y, theta, covariance))

def plot_ekf_results(data):
    if not data:
        print("No EKF data to plot.")
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("EKF XY Position with Covariance")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.axis("equal")

    xs = [d[1] for d in data]
    ys = [d[2] for d in data]
    covs = [d[4] for d in data]

    # Plot trajectory
    ax.plot(xs, ys, 'b-', label='EKF Path', linewidth=1.2)

    for (x, y, cov) in zip(xs, ys, covs):
        cov_xx, cov_yy, cov_xy = cov[0], cov[7], cov[1]
        cov_xy_mat = np.array([[cov_xx, cov_xy],
                               [cov_xy, cov_yy]])
        eigvals, _ = np.linalg.eig(cov_xy_mat)
        radius = math.sqrt(abs(max(eigvals)))  # 1-sigma radius
        scale = 3.0
        radius *= scale
        circ = plt.Circle((x, y), radius, color='g', fill=False, alpha=0.5)
        ax.add_patch(circ)

    # Center around origin
    max_range = max(abs(min(xs + [0])), abs(max(xs + [0])),
                    abs(min(ys + [0])), abs(max(ys + [0])))
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.plot(0, 0, 'ko', label='Origin')

    ax.legend()
    plt.show()

def main():
    data = []
    extract_ekf_data(data)
    print(f"Loaded {len(data)} messages from bag")
    plot_ekf_results(data)

if __name__ == "__main__":
    main()
