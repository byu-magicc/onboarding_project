#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

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


class WallsSensor(Node):

    def __init__(self):
        super().__init__(NodeName)
        self.walls = []
        qos_profile = QoSProfile(
            depth=20,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.walls_loc_sub = self.create_subscription(WallsMsgType, WallsTopicName, self.set_walls, qos_profile)
        self.truth_state_sub = self.create_subscription(TruthMsgType, TruthTopicName, self.truth_callback, 1)
        self.sensor_pub = self.create_publisher(SensorMsgType, SensorTopicName, 10)

    def truth_callback(self, msg):
        sensor_msg = SensorMsgType()
        sensor_msg.data = self.calc_walls_dist(msg.pose.position)
        self.sensor_pub.publish(sensor_msg)

    def calc_walls_dist(self, position):

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
                if ds > 0:
                    dist_south = min(dist_south, ds)

            # If wall in line of sight e/w
            if (abs(x - w.position.x) < (w.dimensions.x / 2)): # Infinite wall height
            # if (abs(x - w.position.x) < (w.dimensions.x / 2)) and (abs(z - w.position.z) < (w.dimensions.z / 2)):
                de = (w.position.y - w.dimensions.y / 2) - y - drone_cylinder_radius
                dw = y - (w.position.y + w.dimensions.y / 2) - drone_cylinder_radius
                if de > 0:
                    dist_east = min(dist_east, de)
                if dw > 0:
                    dist_west = min(dist_west, dw)

        return np.array([dist_north, dist_east, dist_south, dist_west], dtype=np.float32).tolist()

    def set_walls(self, msg):
        w = Wall(msg)
        self.walls.append(w)
        self.walls = sorted(self.walls, key=lambda w: w.marker_id)

class Wall():

    def __init__(self, msg):
        self.set_msg_features(msg)
        self.calc_dimensions()

    def set_msg_features(self, msg):
        self.marker_id = msg.id
        self.marker_type = msg.type
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        self.dimensions = msg.scale
        self.color = msg.color

    def calc_dimensions(self):
        pass

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