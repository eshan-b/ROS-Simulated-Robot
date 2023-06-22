import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt

# Global variables
map_data = np.zeros((800, 800))  # Initialize an empty map


def generate_synthetic_data():
    # Generate synthetic LiDAR scan data
    num_points = 1000
    min_angle = -np.pi/2
    max_angle = np.pi/2
    angles = np.linspace(min_angle, max_angle, num_points)
    ranges = np.random.uniform(0.1, 10.0, num_points)

    # Create a LaserScan message
    scan_msg = LaserScan()
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = 'base_laser_link'
    scan_msg.angle_min = min_angle
    scan_msg.angle_max = max_angle
    scan_msg.angle_increment = (max_angle - min_angle) / (num_points - 1)
    scan_msg.range_min = 0.1
    scan_msg.range_max = 10.0
    scan_msg.ranges = ranges.tolist()

    return scan_msg


def generate_synthetic_pose():
    # Generate synthetic robot pose (odometry data)
    x = np.random.uniform(-10.0, 10.0)
    y = np.random.uniform(-10.0, 10.0)
    yaw = np.random.uniform(-np.pi, np.pi)

    # Create an Odometry message
    pose_msg = Odometry()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'odom'
    pose_msg.pose.pose.position.x = x
    pose_msg.pose.pose.position.y = y
    quaternion = euler_from_quaternion([0.0, 0.0, yaw])
    pose_msg.pose.pose.orientation.x = quaternion[0]
    pose_msg.pose.pose.orientation.y = quaternion[1]
    pose_msg.pose.pose.orientation.z = quaternion[2]
    pose_msg.pose.pose.orientation.w = quaternion[3]

    return pose_msg


def scan_callback(msg):
    global map_data

    # Update map with scan data
    ranges = msg.ranges
    angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
    for i in range(len(ranges)):
        if ranges[i] < msg.range_max:
            x = int(ranges[i] * np.cos(angles[i]) * 20 + 400)
            y = int(ranges[i] * np.sin(angles[i]) * 20 + 400)
            map_data[x, y] = 1


def pose_callback(msg):
    # Extract robot's position from odometry data
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation

    roll, pitch, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])

    # Visualize the map and robot's position
    plt.imshow(map_data, origin='lower', cmap='gray')
    plt.plot(y * 20 + 400, x * 20 + 400, 'ro')  # Assuming 1 unit = 20 cm
    plt.xlim(0, 800)
    plt.ylim(0, 800)
    plt.show()


def main():
    rospy.init_node('slam_demo')
    rospy.Subscriber('/scan', LaserScan, scan_callback)  # LiDAR scan data
    rospy.Subscriber('/odom', Odometry, pose_callback)  # Robot's odometry data

    # Generate synthetic data at a fixed rate
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        scan_msg = generate_synthetic_data()
        pose_msg = generate_synthetic_pose()

        scan_callback(scan_msg)
        pose_callback(pose_msg)

        rate.sleep()


if __name__ == '__main__':
    main()
