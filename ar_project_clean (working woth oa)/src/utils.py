#!/usr/bin/python
# -*- coding: utf-8 -*-

"""Library with helpful functions for the Probabilistic Robotics labs."""

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
                  color=(1, 0, 0),marker_id=0):
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
    msg.scale.x = 0.01
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
        [13.5, 5, 13.5, 13.5],
        [6.5, 5, 13.5, 5],
        [6.25, 13.5, 13.5, 13.5],
        [3, 2.5, 3, 17.5]]).T

    lines[1, :] = -lines[1, :]
    lines[3, :] = -lines[3, :]
    dis = -10#-4.0 + 0.05
    lines = lines * 1.0 + np.array([[dis, -dis, dis, -dis]]).T
    # Transform to specified frame
    lines -= np.array([[x, y, x, y]]).T
    rot = np.array([[np.cos(a), -np.sin(a)],
                    [np.sin(a), np.cos(a)]])
    rotate = np.vstack((np.hstack((rot, np.zeros((2, 2)))),
                        np.hstack((np.zeros((2, 2)), rot))))

    return np.dot(rotate, lines).T


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

########################################################################
def yaw_from_quaternion(quat):
    """
    Extract yaw from a geometry_msgs.msg.Quaternion.

    :param geometry_msgs.msg.Quaternion quat: the quaternion.
    :returns: yaw angle from the quaternion.
    :rtype: :py:obj:`float`
    """
    return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]


########################################################################
def quaternion_from_yaw(yaw):
    """
    Create a Quaternion from a yaw angle.

    :param float yawt: the yaw angle.
    :returns: the quaternion.
    :rtype: :py:obj:`tuple`
    """
    return quaternion_from_euler(0.0, 0.0, yaw)

########################################################################
def get_ekf_msgs(ekf):
    """
    Create messages to visualize EKFs.

    The messages are odometry and uncertainity.

    :param EKF ekf: the EKF filter.
    :returns: a list of messages containing [the odometry of the filter,
        the uncertainty, the translation from origin, the rotation from origin,
        the map lines.
    :rtype: :py:obj:`list`
    """
    # Time
    time = rospy.Time.now()

    # Odometry
    msg_odom = Odometry()
    msg_odom.header.stamp = time
    msg_odom.header.frame_id = 'world'
    msg_odom.pose.pose.position.x = ekf.xk[0]
    msg_odom.pose.pose.position.y = ekf.xk[1]
    msg_odom.pose.pose.position.z = 0
    quat = quaternion_from_yaw(ekf.xk[2])
    msg_odom.pose.pose.orientation.x = quat[0]
    msg_odom.pose.pose.orientation.y = quat[1]
    msg_odom.pose.pose.orientation.z = quat[2]
    msg_odom.pose.pose.orientation.w = quat[3]

    # Uncertainity
    uncert = ekf.Pk[:2, :2].copy()
    val, vec = np.linalg.eigh(uncert)
    yaw = np.arctan2(vec[1, 0], vec[0, 0])
    quat = quaternion_from_yaw(yaw)
    msg_ellipse = Marker()
    msg_ellipse.header.frame_id = "world"
    msg_ellipse.header.stamp = time
    msg_ellipse.type = Marker.CYLINDER
    msg_ellipse.pose.position.x = ekf.xk[0]
    msg_ellipse.pose.position.y = ekf.xk[1]
    msg_ellipse.pose.position.z = -0.1  # below others
    msg_ellipse.pose.orientation.x = quat[0]
    msg_ellipse.pose.orientation.y = quat[1]
    msg_ellipse.pose.orientation.z = quat[2]
    msg_ellipse.pose.orientation.w = quat[3]
    msg_ellipse.scale.x = 2 * math.sqrt(val[0])
    msg_ellipse.scale.y = 2 * math.sqrt(val[1])
    msg_ellipse.scale.z = 0.05
    msg_ellipse.color.a = 0.6
    msg_ellipse.color.r = 0.0
    msg_ellipse.color.g = 0.7
    msg_ellipse.color.b = 0.7

    # TF
    trans = (msg_odom.pose.pose.position.x,
             msg_odom.pose.pose.position.y,
             msg_odom.pose.pose.position.z)
    rotat = (msg_odom.pose.pose.orientation.x,
             msg_odom.pose.pose.orientation.y,
             msg_odom.pose.pose.orientation.z,
             msg_odom.pose.pose.orientation.w)

    # Reconstruct the map to visualize (if is SLAM)
    room_map_polar = np.zeros((0, 2))
    room_map_points = np.zeros((0, 4))
    if hasattr(ekf, 'get_number_of_features_in_map'):
        for i in range(0, ekf.get_number_of_features_in_map()):
            if ekf.featureObservedN.shape[0] == 0 or \
               ekf.featureObservedN[i] >= ekf.min_observations:
                rho = ekf.xk[2*i+3]
                phi = ekf.xk[2*i+4]
                plline = np.array([rho, phi])
                room_map_polar = np.vstack([room_map_polar, plline])
                aux = np.zeros((1, 4))
                if np.abs(np.abs(phi)-np.pi/2) < np.deg2rad(45):
                    # Horizontal line
                    aux[0, 0] = -5
                    aux[0, 2] = 5
                    aux[0, 1] = polar2y(plline, aux[0, 0])
                    aux[0, 3] = polar2y(plline, aux[0, 2])
                else:
                    # Vertical line
                    aux[0, 1] = -5
                    aux[0, 3] = 5
                    aux[0, 0] = polar2x(plline, aux[0, 1])
                    aux[0, 2] = polar2x(plline, aux[0, 3])

                aux[0, 1] = polar2y(plline, aux[0, 0])
                aux[0, 3] = polar2y(plline, aux[0, 2])
                room_map_points = np.vstack([room_map_points, aux])

    return msg_odom, msg_ellipse, trans, rotat, room_map_points


########################################################################
def polar2y(line, x):
    """
    Compute the value of y in a line given x.

    Given a line in polar coordinates and the x value of a point computes
    its y value.

    :param numpy.ndarray line: the line as [rho, theta].
    :param float x: the value in x coordinates.
    :returns: the value in y coordinates.
    :rtype: :py:obj:`float`
    """
    sin = np.sin(line[1])
    cos = np.cos(line[1])
    x0 = line[0] * cos
    y0 = line[0] * sin
    m = -cos/sin
    return m*(x-x0) + y0

########################################################################
def polar2x(line, y):
    """
    Compute the value of y in a line given x.

    Given a line in polar coordinates and the x value of a point computes
    its y value.

    :param numpy.ndarray line: the line as [rho, theta].
    :param float x: the value in x coordinates.
    :returns: the value in y coordinates.
    :rtype: :py:obj:`float`
    """
    sin = np.sin(line[1])
    cos = np.cos(line[1])
    x0 = line[0] * cos
    y0 = line[0] * sin
    m = -cos/sin
    return (y-y0)/m+x0

########################################################################
def get_polar_line(line, odom=[0.0, 0.0, 0.0]):
    """
    Transform a line from cartesian to polar coordinates.

    Transforms a line from [x1 y1 x2 y2] from the world frame to the
    vehicle frame using odomotrey [x y ang].

    By default only transforms line to polar without translation.

    :param numpy.ndarray line: line as [x1 y1 x2 y2].
    :param list odom: the origin of the frame as [x y ang].
    :returns: the polar line as [range theta].
    :rtype: :py:obj:`numpy.ndarray`
    """
    # Line points
    x1 = line[0]
    y1 = line[1]
    x2 = line[2]
    y2 = line[3]

    # Compute line (a, b, c) and range
    line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
    pt = np.array([odom[0], odom[1], 1])
    dist = np.dot(pt, line) / np.linalg.norm(line[:2])

    # Compute angle
    if dist < 0:
        ang = np.arctan2(line[1], line[0])
    else:
        ang = np.arctan2(-line[1], -line[0])

    # Return in the vehicle frame
    return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
