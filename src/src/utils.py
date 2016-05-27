#!/usr/bin/python
# -*- coding: utf-8 -*-

"""Library with helpful functions""" 

# ROS init
import rospy

# Math
import math
import numpy as np

# ROS messages
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose

# Transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

###############################################################################
def publish_lines(lines, pub, frame='world', ns='none', time=None,
                  color=(1, 0, 0),marker_id=0, thickness=0.01):
    """
    Publish lines from an array of shape (N, 4) as a Marker message.

    N the number of lines in the array. Lines are represented by the start and
    end points as [x1 y1 x2 y2].

    :param numpy.ndarray lines: the lines as rows [x1, y1, x2, y2].
    :param rospy.Publisher pub: ROS publisher for Marker messages.
    :param str frame: the frame of the published message.
    :param str ns: namespace of the published message.
    :param rospy.Time time: the timestamp of the published message.
    :param tuple color: RGB tuple defining the color of the published message.
    """
    # Create message
    msg = Marker()
    msg.header.stamp = time if time is not None else rospy.Time.now()
    msg.header.frame_id = frame
    msg.ns = ns
    msg.id = marker_id
    msg.type = msg.LINE_LIST
    msg.action = msg.ADD
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    msg.scale.x = thickness
    msg.scale.y = 0.0
    msg.scale.z = 0.0
    msg.color.r = color[0]
    msg.color.g = color[1]
    msg.color.b = color[2]
    msg.color.a = 1.0
    for i in range(lines.shape[0]):
        msg.points.append(Point(lines[i, 0], lines[i, 1], 0))
        msg.points.append(Point(lines[i, 2], lines[i, 3], 0))
    # Publish
    pub.publish(msg)


########################################################################
def get_map(x=0, y=0, a=0):
    """
    Retrieve the map for dataset3 with offsets [x y a] if necessary.

    Lines defined as [x1 y1 x2 y2].

    For the EKF lab use: x = 0.7841748 y = 0.313926 a = -0.03

    This is the map for dataset1.bag

    :param float x: initial x position of the robot in the map.
    :param float y: initial y position of the robot in the map.
    :param float a: initial orientation of the robot in the map.
    :returns: the lines defined by rows [x1, y1, x2, y2].
    :rtype: :py:obj:`numpy.ndarray`
    """
    lines = np.array([
        [0, 0, 0, 20],
        [0, 0, 20, 0],
        [20, 0, 20, 20],
        [20, 20, 0, 20],
        [14.25, 5.5, 14.25, 14],
        [7.25, 5.5, 14.25, 5.5],
        [7, 14, 14.25, 14],

        [3.5, 2.5, 3.5, 17.5]]).T

    lines[1, :] = -lines[1, :]
    lines[3, :] = -lines[3, :]
    dis = -10
    lines = lines * 1.0 + np.array([[dis, -dis, dis, -dis]]).T
    # Transform to specified frame
    lines -= np.array([[x, y, x, y]]).T
    rot = np.array([[np.cos(a), -np.sin(a)],
                    [np.sin(a), np.cos(a)]])
    rotate = np.vstack((np.hstack((rot, np.zeros((2, 2)))),
                        np.hstack((np.zeros((2, 2)), rot))))

    return np.dot(rotate, lines).T

########################################################################

def get_map_udg(x=0, y=0, a=0):
    """
    Retrieve the map for UDG with offsets [x y a] if necessary.

    Lines defined as [x1 y1 x2 y2].

    This is the map for UDG 

    :param float x: initial x position of the robot in the map.
    :param float y: initial y position of the robot in the map.
    :param float a: initial orientation of the robot in the map.
    :returns: the lines defined by rows [x1, y1, x2, y2].
    :rtype: :py:obj:`numpy.ndarray`
    """
    lines = np.array([
        [0, 0, 0, 40],
        [0, 0, 40, 0],
        [40, 0, 40, 40],
        [0, 40, 40, 40],
        # U
        [7, 9, 13, 9],
        [13, 9, 13, 15],
        [13, 15, 7, 15],
        # G
        [16, 34, 16, 22],
        [16, 22, 31, 22],
        [31, 22, 31, 34],
        [31, 34, 24, 34],
        [24, 34, 24, 28],
        #D
        [34, 7, 22, 7],
        [22, 7, 22, 15],
        [22, 15, 24, 17],
        [24, 17, 32, 17],
        [32, 17, 34, 15],
        [34, 15, 34, 11]
        ]).T

    lines[1, :] = -lines[1, :]
    lines[3, :] = -lines[3, :]
    dis = -20
    lines = lines * 1.0 + np.array([[dis, -dis, dis, -dis]]).T
    # Transform to specified frame
    lines -= np.array([[x, y, x, y]]).T
    rot = np.array([[np.cos(a), -np.sin(a)],
                    [np.sin(a), np.cos(a)]])
    fl = np.array([[1, 0],[0, -1]])
    rotate = np.vstack((np.hstack((rot, np.zeros((2, 2)))),
                        np.hstack((np.zeros((2, 2)), rot))))
    flip = np.vstack((np.hstack((fl, np.zeros((2, 2)))),
                        np.hstack((np.zeros((2, 2)), fl))))

    return np.dot(flip, np.dot(rotate, lines)).T

########################################################################

def angle_wrap(ang):
    """
    Return the angle normalized between [-pi, pi].

    Works with numbers and numpy arrays.

    :param ang: the input angle/s.
    :type ang: float, numpy.ndarray
    :returns: angle normalized between [-pi, pi].
    :rtype: float, numpy.ndarray
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang

