#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Trigger

import tf_transformations
import math
import numpy as np
from .algorithms.path_finder import PathFinder
import time

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1


class PathFinderController(Node):
    def __init__(self):

        ## Initialisation du Node
        super().__init__('simple_mapper_multiranger')
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value
        self.declare_parameter('max_turn_rate', 0.5)
        max_turn_rate = self.get_parameter('max_turn_rate').value
        self.declare_parameter('max_forward_speed', 0.5)
        max_forward_speed = self.get_parameter('max_forward_speed').value
        self.declare_parameter('wall_following_direction', 'right')
        self.wall_following_direction = self.get_parameter('wall_following_direction').value

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)

        # add service to stop wall following and make the crazyflie land
        self.srv = self.create_service(Trigger, robot_prefix + '/stop_wall_following', self.stop_wall_following_cb)

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]

        self.position_update = False

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(f"Wall following set for crazyflie " + robot_prefix +
                               f" using the scan topic with a delay of {self.delay} seconds")
        
        ## Fin de l'initialisation du Node

        # Create a timer to run the wall following state machine
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Initialize wall following state machine
        self.wall_following = PathFinder()

        # Give a take off command but wait for the delay to start the wall following
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        msg = Twist()
        msg.linear.z = 1.
        self.twist_publisher.publish(msg)

    def stop_wall_following_cb(self, request, response):
        self.get_logger().info('Stopping wall following')
        self.timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = -0.2
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

        response.success = True

        return response

    # Fonctione appeler a chaque fois que le timer est declenche
    def timer_callback(self):

        # On attend le temps de delay avant de commencer le path finding
        if self.wait_for_start:
            if self.get_clock().now().nanoseconds * 1e-9 - self.start_clock > self.delay:
                self.get_logger().info('Starting wall following')
                self.wait_for_start = False
            else:
                return

        # initialize variables
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0

        # Get position
        x_pos = self.position[0]
        y_pos = self.position[1]
        #print(f"== DEBUG ==> Position: {round(x_pos, 2)}, {round(y_pos, 2)}")

        # Get Yaw
        actual_yaw_rad = self.angles[2]

        # get velocity commands and current state from wall following state machine
        velocity_x, velocity_y, yaw_rate = self.wall_following.path_finder(x_pos, y_pos, actual_yaw_rad)
        #print(f"== DEBUG ==> path finder: {round(velocity_x, 2)}, {round(velocity_y, 2)}, {round(yaw_rate, 2)}")

        msg = Twist()
        msg.linear.x = velocity_x
        msg.linear.y = velocity_y
        msg.angular.z = yaw_rate
        self.twist_publisher.publish(msg)

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges

def main(args=None):
    rclpy.init(args=args)
    wall_following_multiranger = PathFinderController()
    rclpy.spin(wall_following_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
