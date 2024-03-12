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


path_to_directory = '/sim_ws/src/f1tenth_lab5_sub/waypoints'
os.makedirs(path_to_directory, exist_ok=True)

file_path = os.path.join(path_to_directory, 'waypoints2.csv')
file = open(file_path, 'w')

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.save_waypoint,
            10)
        self.subscription

    def save_waypoint(self, data):

        file.write('%f, %f\n' % (data.pose.pose.position.x,
                                         data.pose.pose.position.y))

def shutdown():
    file.close()

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    atexit.register(shutdown)
    print(f'saving waypoints to {file_path}')
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
