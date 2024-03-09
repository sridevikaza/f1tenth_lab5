#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
import tf_transformations
from nav_msgs.msg import Odometry
import os
from time import gmtime, strftime
from numpy import linalg as LA


# Specify your absolute path here
path_to_directory = '/sim_ws/src/f1tenth_lab5_sub/waypoints'

# Ensure the directory exists
os.makedirs(path_to_directory, exist_ok=True)

file_path = os.path.join(path_to_directory, 'waypoints.csv')
# file_path = os.path.join(path_to_directory, strftime('wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv')
file = open(file_path, 'w')

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.save_waypoint,
            10)
        self.subscription  # prevent unused variable warning

    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x,
                               data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z,
                               data.pose.pose.orientation.w])

        euler = tf_transformations.euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([data.twist.twist.linear.x,
                                  data.twist.twist.linear.y,
                                  data.twist.twist.linear.z]), 2)

        if data.twist.twist.linear.x > 0.0:
            self.get_logger().info(str(data.twist.twist.linear.x))

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                         data.pose.pose.position.y,
                                         euler[2],
                                         speed))

def shutdown():
    file.close()
    print('Goodbye')

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    atexit.register(shutdown)
    print(f'Saving waypoints to {file_path}')
    rclpy.spin(waypoint_logger)

    waypoint_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
