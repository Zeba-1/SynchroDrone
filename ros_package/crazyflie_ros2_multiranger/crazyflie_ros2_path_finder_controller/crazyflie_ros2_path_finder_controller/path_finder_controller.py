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

        self.declare_parameter('robot_prefix', ['/crazyflie0'])
        drones = self.get_parameter('robot_prefix').value
        self.get_logger().info(f"== DEBUG ==> drones: {drones}")
        self.nb_drones = len(drones)

        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value
        self.declare_parameter('max_turn_rate', 0.5)
        max_turn_rate = self.get_parameter('max_turn_rate').value
        self.declare_parameter('max_forward_speed', 0.5)
        max_forward_speed = self.get_parameter('max_forward_speed').value
        self.declare_parameter('wall_following_direction', 'right')
        self.wall_following_direction = self.get_parameter('wall_following_direction').value


        self.position = []
        self.angles = []
        self.twist_publisher = []
        self.odom_subscriber = []
        for robot_prefix in drones:
            robot_index = int(robot_prefix[-1])
            self.get_logger().info(f"== DEBUG ==> {robot_prefix}/odom !")
            if robot_index == 0:
                self.odom_subscriber.append(self.create_subscription(Odometry, f"{robot_prefix}/odom", self.odom_subscribe_callback, 10))
            else:
                self.odom_subscriber.append(self.create_subscription(Odometry, f"{robot_prefix}/odom", self.odom_subscribe_callback1, 10))
            
            self.twist_publisher.append(self.create_publisher(Twist, f'/cmd_vel{robot_index}', 10))

            # add service to stop wall following and make the crazyflie land
            self.srv = self.create_service(Trigger, robot_prefix + '/stop_wall_following', self.stop_wall_following_cb)

            self.position.append([0.0, 0.0, 0.0])
            self.angles.append([0.0, 0.0, 0.0])

        self.position_update = False
        
        ## Fin de l'initialisation du Node

        # Initialize wall following state machine
        self.wall_following = PathFinder(self.nb_drones, self.get_logger())
        self.get_logger().info(f"== DEBUG ==> Pathfinding completed")

        # Create a timer to run the wall following state machine
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Give a take off command but wait for the delay to start the wall following
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        msg = Twist()
        msg.linear.z = 1.
        for pub in self.twist_publisher:
            pub.publish(msg)

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

        positions = []
        for i in range(self.nb_drones):
            # Get position
            x_pos = self.position[i][0]
            y_pos = self.position[i][1]

            # Get Yaw
            actual_yaw_rad = self.angles[i][2]

            positions.append((x_pos, y_pos, actual_yaw_rad))


        # get velocity commands and current state from wall following state machine
        velocities = self.wall_following.path_finder(positions)

        for i in range(self.nb_drones):
            x_vel, y_vel, yaw_rate = velocities[i]
            msg = Twist()
            msg.linear.x = x_vel
            msg.linear.y = y_vel
            msg.angular.z = yaw_rate
            self.twist_publisher[i].publish(msg)

    def odom_subscribe_callback(self, msg):
#        self.get_logger().info(f"== DEBUG ==> Odom received for {0} : {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        self.position[0][0] = msg.pose.pose.position.x
        self.position[0][1] = msg.pose.pose.position.y
        self.position[0][2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0][0] = euler[0]
        self.angles[0][1] = euler[1]
        self.angles[0][2] = euler[2]
        self.position_updated = True

    def odom_subscribe_callback1(self, msg):
#        self.get_logger().info(f"== DEBUG ==> Odom received for {1} : {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        self.position[1][0] = msg.pose.pose.position.x
        self.position[1][1] = msg.pose.pose.position.y
        self.position[1][2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[1][0] = euler[0]
        self.angles[1][1] = euler[1]
        self.angles[1][2] = euler[2]
        self.position_updated = True

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
