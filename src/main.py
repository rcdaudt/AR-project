#!/usr/bin/python
# -*- coding: utf-8 -*-

"""Main node that connects to the necessary topics."""

# Basic ROS
import rospy
import tf

# ROS messages
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# Maths
import numpy as np

# Custom libraries
import probabilistic_lib.functions as funcs

# Import classes

# e.g. Extended Kalman Filter
# from ekf_localization import EKF


# ==============================================================================
class ControllerNode(object):
    """Class to hold all ROS related transactions."""

    # ==========================================================================
    def __init__(self):
        """Initialize publishers, subscribers and the classes."""

        # Init publishers
        self.loadMap = rospy.Subscriber("map", OccupancyGrid, self.showMap)
        self.pubMap = rospy.Publisher("map1", OccupancyGrid, queue_size=2)
        # Init subscribers
        # e.g. self.sensor = rospy.Subscriber("sensor", dataType, self.callbackFunctionForSensorNode)

        # Init additional classes and modules
        # e.g. self.ekf = EKF(xinit, odom_lin_sigma, odom_ang_sigma, meas_rng_noise,
        #              meas_ang_noise)

    # ==========================================================================
    # define callback functions in this block
    def showMap(self, msg):
        self.pubMap.publish(msg)
    # ==========================================================================
    def iterate(self):
        """Main loop"""

    # ==========================================================================
    def publish_results(self):

        """Publish results (if needed) to rviz"""

if __name__ == '__main__':

    # ROS initializzation
    rospy.init_node('main')
    node = ControllerNode()
    r = rospy.Rate(10)

    # Main loop
    while not rospy.is_shutdown():
        node.iterate()
        r.sleep()
