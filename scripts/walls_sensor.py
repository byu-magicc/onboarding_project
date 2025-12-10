#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.clock import Clock

from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from rosflight_msgs.msg import SimState

NodeName = 'walls_sensor'
WallsMsgType = Marker
WallsTopicName = 'rviz/hallway'
TruthMsgType = SimState
TruthTopicName = 'sim/truth_state'
SensorMsgType = Float32MultiArray
SensorTopicName = 'sensors/walls_sensor'

drone_cylinder_radius = 1.
chalk_circle_radius = 5.
height_threshold = 10.

class WallsSensor(Node):

    def __init__(self):
        super().__init__(NodeName)
        self.begun = False
        self.clock = self.get_clock()
        self.init_time = self.clock.now()
        self.min_distance = []
        self.walls = []
        self.karl = None
        self.finished = False
        # Quality of service profile to collect walls
        qos_profile = QoSProfile(
            depth=20,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # Publisher and subscribers
        self.walls_loc_sub = self.create_subscription(WallsMsgType, WallsTopicName, self.collect_markers, qos_profile)
        self.truth_state_sub = self.create_subscription(TruthMsgType, TruthTopicName, self.truth_callback, 1)
        self.sensor_pub = self.create_publisher(SensorMsgType, SensorTopicName, 10)

    def truth_callback(self, msg):
        sensor_msg = SensorMsgType()
        sensor_msg.data = self.calc_walls_dist(msg.pose.position)
        self.sensor_pub.publish(sensor_msg)

    def calc_walls_dist(self, position):

        collision_detected = False

        x = position.x
        y = position.y
        z = position.z

        dist_north, dist_east, dist_south, dist_west = np.inf, np.inf, np.inf, np.inf
        for w in self.walls:

            # If wall in line of sight n/s
            if (abs(y - w.position.y) < (w.dimensions.y / 2)): # Infinite wall height
            # if (abs(y - w.position.y) < (w.dimensions.y / 2)) and (abs(z - w.position.z) < (w.dimensions.z / 2)):
                dn = (w.position.x - w.dimensions.x / 2) - x - drone_cylinder_radius
                ds = x - (w.position.x + w.dimensions.x / 2) - drone_cylinder_radius
                if dn > 0:
                    dist_north = min(dist_north, dn)
                elif dn + drone_cylinder_radius > 0:
                    collision_detected = True
                if ds > 0:
                    dist_south = min(dist_south, ds)
                elif ds + drone_cylinder_radius > 0:
                    collision_detected = True

            # If wall in line of sight e/w
            if (abs(x - w.position.x) < (w.dimensions.x / 2)): # Infinite wall height
            # if (abs(x - w.position.x) < (w.dimensions.x / 2)) and (abs(z - w.position.z) < (w.dimensions.z / 2)):
                de = (w.position.y - w.dimensions.y / 2) - y - drone_cylinder_radius
                dw = y - (w.position.y + w.dimensions.y / 2) - drone_cylinder_radius
                if de > 0:
                    dist_east = min(dist_east, de)
                elif de + drone_cylinder_radius > 0:
                    collision_detected = True
                if dw > 0:
                    dist_west = min(dist_west, dw)
                elif dw + drone_cylinder_radius > 0:
                    collision_detected = True

        nesw = np.array([dist_north, dist_east, dist_south, dist_west], dtype=np.float32).tolist()

        # If in a trial
        if self.begun and not self.finished:
            if collision_detected:
                self.run_wall_collision_condition()
            else:
                self.min_distance.append([self.calc_time_passed(), min(nesw)])
                if self.check_if_reached_karl(position):
                    self.run_finished_condition()
        # If waiting to begin
        elif not self.begun:
            if self.check_if_near_zero(x, y):
                self.run_begin_condition()
            else:
                self.get_logger().info('You must be near (0, 0) to begin timing.')

        return nesw

    def check_if_near_zero(self, x, y, close_threshold=5.):
        return np.linalg.norm([x, y]) < close_threshold

    def run_begin_condition(self):
        self.begun = True
        self.init_time = self.clock.now()
        self.min_distance = []
        self.finished = False

    def run_finished_condition(self, success=True):
        self.finished = True
        self.elapsed_time = self.calc_time_passed()
        self.destroy_subscription(self.truth_state_sub)
        min_distance_array = np.array(self.min_distance)
        if success:
            self.get_logger().info(f'Karl has received the chalkolate milk in {self.elapsed_time} seconds!')
            self.get_logger().info(f'Minimum distance to walls: {min(min_distance_array[:, 1])}')
        self.plot_distance_to_walls(min_distance_array)

    def run_wall_collision_condition(self):
        self.get_logger().info('Mission failed! Hit a wall!')
        self.run_finished_condition(success=False) # This is a temporary way to end the simulation. Eventually, I want it to reset when the drone goes back to the start.

    def calc_time_passed(self):
        elapsed_time = self.clock.now() - self.init_time
        elapsed_time_sec = elapsed_time.to_msg().sec
        elapsed_time_nano = elapsed_time.to_msg().nanosec
        timestamp = float(elapsed_time_sec) + (elapsed_time_nano * 1e-9)
        return timestamp

    def check_if_reached_karl(self, position):
        if self.karl is None:
            return False
        position_array = np.array([position.x, position.y, position.z])
        if (np.linalg.norm(self.karl.position_array[:2] - position_array[:2]) < chalk_circle_radius) and (abs(position.z - self.karl.position.z) < height_threshold):
            return True
        else:
            return False

    def plot_distance_to_walls(self, min_distance_array):
        plt.plot(min_distance_array[:, 0], min_distance_array[:, 1])
        plt.title('Distance to Nearest Wall')
        plt.show()

    def collect_markers(self, msg):
        m = PythonMarker(msg)
        if msg.type == 1:
            self.walls.append(m)
            self.walls = sorted(self.walls, key=lambda w: w.marker_id)
        elif msg.type == 10:
            self.karl = m

class PythonMarker():

    def __init__(self, msg):
        self.set_msg_features(msg)

    def set_msg_features(self, msg):
        self.marker_id = msg.id
        self.marker_type = msg.type
        self.position = self.make_ned(msg.pose.position)
        self.position_array = np.array([self.position.x, self.position.y, self.position.z])
        self.orientation = msg.pose.orientation
        self.dimensions = msg.scale
        self.color = msg.color

    def make_ned(self, position):
        position.y *= -1
        position.z *= -1
        return position

    def __str__(self):
        to_print = ''
        to_print += f'Marker ID:    {self.marker_id}\n'
        to_print += f'Marker Type:  {self.marker_type}\n'
        to_print += f'Position:     {self.position}\n'
        to_print += f'Orientation:  {self.marker_id}\n'
        to_print += f'Dimensions:   {self.dimensions}\n'
        return to_print


def main(args=None):
    rclpy.init(args=args)
    node = WallsSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()