#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node

from roscopter_msgs.srv import AddWaypoint
from roscopter_msgs.msg import Waypoint
from roscopter_msgs.msg import State
from std_msgs.msg import Float32MultiArray

NodeName = 'navigation'

class Navigation(Node):

    def __init__(self):
        super().__init__(NodeName)
        self.dist_north = 0.0
        self.dist_east = 0.0
        self.dist_south = 0.0
        self.dist_west = 0.0
        self.curr_north = 0.0
        self.curr_east = 0.0
        self.curr_down = 0.0
        self.next_north = 0.0
        self.next_east = 0.0
        self.axis = 'east'

        self.Wall_sense_sub = self.create_subscription( # subscribe to wall sensors
            Float32MultiArray,
            'sensors/walls_sensor',
            self.sensor_callback,
            10)
        self.state_sub = self.create_subscription(      # subscribe to estimator
            State,
            'estimated_state',
            self.state_callback,
            10)
        self.add_waypoint_client = self.create_client(  # create client for AddWaypoint service
            AddWaypoint,
            'path_planner/add_waypoint')
        while not self.add_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('add waypoint service not available, waiting again...')

    def sensor_callback(self, msg):
        self.dist_east = msg.data[1]
        self.dist_north = msg.data[0]
        self.dist_south = msg.data[2]
        self.dist_west = msg.data[3]
        #self.get_logger().info(    # uncomment to see live sensor data
        #    'Wall Distance: N: %.2f E: %.2f S: %.2f W: %.2f' %
        #    (self.dist_north, self.dist_east, self.dist_south, self.dist_west)
        #)

    def calculate_waypoint(self):
        while (self.dist_north == 0.0): # Make sure sensors are loaded in
            time.sleep(1)
            self.get_logger().info('Waiting for wall sensors...')
            return
        # if looking for a n/s direction, find the furthest non-infinite wall and set a waypoint 4m away
        if (self.axis != 'north'):
            if ((self.dist_north > self.dist_south) or (self.dist_south > 9999)):
                n_pos = self.curr_north + self.dist_north - 4.0
            else: n_pos = self.curr_north - self.dist_south + 4.0
            self.axis = 'north'
            e_pos = self.curr_east
        elif (self.axis != 'east'): # if looking for a e/w direction set waypoint
            if ((self.dist_east > self.dist_west) or (self.dist_west > 9999)):
                e_pos = self.curr_east + self.dist_east - 4.0
            else: e_pos = self.curr_east - self.dist_west + 4.0
            self.axis = 'east'
            n_pos = self.curr_north
        else: self.get_logger().info('ERROR -- invalid axis variable set')
        self.next_north = n_pos
        self.next_east = e_pos

    def create_waypoint(self):
        self.get_logger().info('Creating Waypoint...')

        self.calculate_waypoint()

        req = AddWaypoint.Request()
        wp_msg = Waypoint()
        wp_msg.type = 1
        wp_msg.w = [self.next_north, self.next_east, -10.0]
        wp_msg.psi = 0.0
        wp_msg.speed = 40.0
        wp_msg.clear_wp_list = False

        req.wp = wp_msg
        req.publish_now = True

        self.get_logger().info(f"Flying to: N: {self.next_north:.2f} E: {self.next_east:.2f}")
        self.add_waypoint_client.call_async(req)

    def check_pos(self, dist_thresh):   # check if the next waypoint has been reached
        if ((self.curr_north < (self.next_north + dist_thresh)) and (self.curr_north > (self.next_north - dist_thresh))
            and (self.curr_east < (self.next_east + dist_thresh)) and (self.curr_east > (self.next_east - dist_thresh))):
            self.get_logger().info('Reached Waypoint')
            self.create_waypoint()

    def state_callback(self, msg):
        self.curr_north = msg.p_n
        self.curr_east = msg.p_e
        self.curr_down = msg.p_d
        self.get_logger().info('Current Position: N: %.2f E: %.2f D: %.2f' %
            (self.curr_north, self.curr_east, self.curr_down)
        )
        self.check_pos(.5)

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()