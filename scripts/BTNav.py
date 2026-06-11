#!/usr/bin/env python3

import py_trees
import rclpy
#import typing
from rclpy.node import Node

from roscopter_msgs.srv import AddWaypoint
from roscopter_msgs.msg import Waypoint
from roscopter_msgs.msg import State
from std_msgs.msg import Float32MultiArray

NodeName = 'navigation'

# Behavior Tree Behaviors

class CheckPos(py_trees.behaviour.Behaviour): # check if the next waypoint has been reached
    def __init__(self, name: str, node: Node) -> None:
        super(CheckPos, self).__init__(name)
        self.node = node

    def initialise(self) -> None:
        self.dist_thresh = 0.5
        self.node.get_logger().info('Checking Position...')

    def update(self) -> py_trees.common.Status:
        if ((self.node.curr_north < (self.node.next_north + self.dist_thresh)) and 
            (self.node.curr_north > (self.node.next_north - self.dist_thresh)) and 
            (self.node.curr_east < (self.node.next_east + self.dist_thresh)) and 
            (self.node.curr_east > (self.node.next_east - self.dist_thresh))):
            
            self.node.get_logger().info('Reached Waypoint')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

class WallSense(py_trees.behaviour.Behaviour): # Make sure sensors are reading proper values
    def __init__(self, name: str, node: Node) -> None:
        super(WallSense, self).__init__(name)
        self.node = node

    def initialise(self) -> None:
        self.node.get_logger().info('Waiting for wall sensors...')

    def update(self) -> py_trees.common.Status:
        if (self.node.dist_north == 0.0):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

# if looking for a n/s direction, find the furthest non-infinite wall and set a waypoint 4m away
class NSWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node) -> None:
        super(NSWaypoint, self).__init__(name)
        self.node = node

    def update(self) -> py_trees.common.Status:
        if (self.node.axis != 'north'):
            if ((self.node.dist_north > self.node.dist_south) or (self.node.dist_south > 9999)):
                self.node.next_north = self.node.curr_north + self.node.dist_north - 4.0
            else:
                self.node.next_north = self.node.curr_north - self.node.dist_south + 4.0
            self.node.axis = 'north'
            self.node.next_east = self.node.curr_east
            self.node.create_waypoint()
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

# if looking for a e/w direction, find the furthest non-infinite wall and set a waypoint 4m away
class EWWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node) -> None:
        super(EWWaypoint, self).__init__(name)
        self.node = node

    def update(self) -> py_trees.common.Status:
        if (self.node.axis != 'east'): 
            if ((self.node.dist_east > self.node.dist_west) or (self.node.dist_west > 9999)):
                self.node.next_east = self.node.curr_east + self.node.dist_east - 4.0
            else:
                self.node.next_east = self.node.curr_east - self.node.dist_west + 4.0
            self.node.axis = 'east'
            self.node.next_north = self.node.curr_north
            self.node.create_waypoint()
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

# ROSflight Node & Tree Execution

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

        # Subscriptions & Clients
        # subscribe to wall sensors
        self.Wall_sense_sub = self.create_subscription(Float32MultiArray, 'sensors/walls_sensor', self.sensor_callback, 10)
        # subscribe to the estimator
        self.state_sub = self.create_subscription(State, 'estimated_state', self.state_callback, 10)
        # create client for AddWaypoint service
        self.add_waypoint_client = self.create_client(AddWaypoint, 'path_planner/add_waypoint')
        
        while not self.add_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('add waypoint service not available, waiting again...')
        
        # Build Behavior Tree
        self.setup_behavior_tree()

        # Create a ROS timer to tick the behavior tree
        self.tree_timer = self.create_timer(0.5, self.tick_tree)

    def setup_behavior_tree(self):
        root = py_trees.composites.Sequence("Sequence", memory=False)

        check_pos = CheckPos(name="Check Pos", node=self)
        wall_sense = WallSense(name="Wall Sense", node=self)

        # the selector create_waypoint checks the North/South direction, then the East/West direction for a position for a new waypoint.
        create_waypoint = py_trees.composites.Selector("Create Waypoint", memory=False)
        ns_waypoint = NSWaypoint(name="NS Waypoint", node=self)
        ew_waypoint = EWWaypoint(name="EW Waypoint", node=self)
        create_waypoint.add_children([ns_waypoint, ew_waypoint])

        root.add_children([check_pos, wall_sense, create_waypoint])

        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        self.behaviour_tree.setup(timeout=15)
        #self.get_logger().info("Behavior Tree Setup Complete.")

    def tick_tree(self):
        # This function runs every 0.5 seconds via the ROS Timer
        self.behaviour_tree.tick()
        # Print the tree status to the console
        #print(py_trees.display.unicode_tree(root=self.behaviour_tree.root, show_status=True))

    def sensor_callback(self, msg):
        self.dist_east = msg.data[1]
        self.dist_north = msg.data[0]
        self.dist_south = msg.data[2]
        self.dist_west = msg.data[3]

    def create_waypoint(self):
        self.get_logger().info('Creating Waypoint...')

        req = AddWaypoint.Request()
        wp_msg = Waypoint()
        wp_msg.type = 1
        wp_msg.w = [self.next_north, self.next_east, -6.0]
        wp_msg.psi = 0.0
        wp_msg.speed = 40.0
        wp_msg.clear_wp_list = False

        req.wp = wp_msg
        req.publish_now = True

        self.get_logger().info(f"Flying to: N: {self.next_north:.2f} E: {self.next_east:.2f}")
        self.add_waypoint_client.call_async(req)

    def state_callback(self, msg):
        self.curr_north = msg.p_n
        self.curr_east = msg.p_e
        self.curr_down = msg.p_d

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()