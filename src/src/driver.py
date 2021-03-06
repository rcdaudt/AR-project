#!/usr/bin/env python

import os

# ROS general imports
import roslib
roslib.load_manifest('ar_project')
import rospy

# Other imoprts
import math #math library
import numpy as np #numpy library

from rrt import rrt
from scipy.ndimage import imread
from splitandmerge import splitandmerge
import utils

#import the library to compute transformations
from tf.transformations import euler_from_quaternion
import tf

#ROS messages
#import appropiate ROS messages
from geometry_msgs.msg import Twist # For
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class driver(object):
    
    def __init__(self):
        '''
        constructor
        '''

        print('Driver constructor')


        self.scale = 0.05
        self.offset_x = 200 # 400
        self.offset_y = 200 # 400

        #Initialize ros node
        rospy.init_node('turtlebot_driver')
        
        #Starting position: in pixels
        self.x_start = rospy.get_param('cur_pos_x', self.offset_x)
        self.y_start = rospy.get_param('cur_pos_y', self.offset_y)
        self.theta_start = rospy.get_param('cur_pos_theta', 0)

        #Initialize goals
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])

        
        #Threshold for distance to goal
        self.goal_th_xy = rospy.get_param('goal_thershold_xy',0.1) #Position threshold
        self.goal_th_ang = rospy.get_param('goal_threshold_ang',0.001) #Orientation threshold
        
        #Point to the first goal
        self.active_goal = 0

        #Initialize number of goals
        self.num_goals = 1

        #Has the goal been loaded?
        self.params_loaded = False

        # For rviz plotting lines
        self.tfBroad = tf.TransformBroadcaster()
        self.i = 0 

       

        #Some parameters for LaserScan information
        self.sensor_spacing = 5 
        self.sensor_max_range = 10
        
        # Define publisher        
        self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=100)
        self.pub_line = rospy.Publisher("lines", Marker,queue_size=0)     
        self.pub_map = rospy.Publisher("linesekf", Marker, queue_size=2)
        self.pub_traj = rospy.Publisher("trajectory", Marker, queue_size=2)

        # Define subscriber
        ##################################################################
        
        self.sub_sensor = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)

        #define the velocity message
        self.vmsg = Twist()

        # Initialize robot's position (in meters)
        self.position_x = rospy.get_param('cur_pos_world_x', 0)
        self.position_y = rospy.get_param('cur_pos_world_y', 0)
        self.position_theta = rospy.get_param('cur_pos_world_theta', 0)

        #Controller parameters
        self.kp_v = 0.1
        self.kp_w = 1.5
        self.kd_v = 0
        self.kd_w = 0
        # self.ki_w = 0.001

        #Controller variables
        self.d_prev = 0
        self.dt_prev = 0
        
        # Obstacle avoidance factors
        self.oa_v = 1
        self.oa_w = 0
        #self.counter = 0

        # Publish map lines
        self.map = utils.get_map() # get_map_udg()
        self.trajectory = np.zeros((0, 4))

        
    def print_goals(self):
        '''
        print_goals prints the list of goals
        '''
        rospy.loginfo("List of goals:")
        rospy.loginfo("X:\t " + str(self.x))
        rospy.loginfo("Y:\t " + str(self.y))
        rospy.loginfo("Theta:\t " + str(self.theta))
    
    def print_goal(self):
        '''
        pritn_goal prints the next goal
        '''
        rospy.loginfo( "Goal: (" + str(self.x[self.active_goal]) + " , " + str(self.y[self.active_goal]) + " , " + str(self.theta[self.active_goal]) + ")")
        
    def print_pose(self):
        '''
        print_pose pints the robot's actual position
        '''
        rospy.loginfo( "Pose: (" + str(self.position_x) + " , " + str(self.position_y) + " , " + str(self.position_theta) + " )")
        
    def print_goal_and_pose(self):
        '''
        pritn_goal_and_pose prints both goal and pose
        '''
        rospy.loginfo("\tPose\t\tGoal")
        rospy.loginfo("X:\t%f\t%f",self.position_x,self.x[self.active_goal])
        rospy.loginfo("Y:\t%f\t%f",self.position_y,self.y[self.active_goal])
        rospy.loginfo("A:\t%f\t%f",self.position_theta,self.theta[self.active_goal])
        
    def dist_to_goal_xy(self):
        '''
        dist_to_goal_xy computes the distance in x and y direction to the 
        active goal
        '''
        return math.sqrt(pow(self.position_x-self.x[self.active_goal],2)+pow(self.position_y-self.y[self.active_goal],2))
    
    def dist_to_goal_ang(self):
        '''
        dist_to_goal_ang computes the orientation distance to the active
        goal
        '''
        return np.abs(self.angle_wrap(self.theta[self.active_goal]-self.position_theta))
        
    def has_arrived_xy(self):
        '''
        has_arrived_xy returns true if the xy distance to the ative goal is
        smaller than the position threshold
        '''
        return self.dist_to_goal_xy()<self.goal_th_xy
        
    def has_arrived_ang(self):
        '''
        has_arrived_ang returns true if the orientation distance to the 
        ative goal is smaller than the orientation threshold
        '''
        return self.dist_to_goal_ang()<self.goal_th_ang
        
    def has_arrived(self):
        '''
        has_arrived return true if the robot is closer than the apropiate 
        threshold to the goal in position and orientation
        '''
        return (self.has_arrived_xy() and self.has_arrived_ang())   
    
    def check_goal(self):
        '''
        check_goal checks if the robot has arrived to the active goal, 
        '''
        if self.has_arrived():
            self.next_goal()
        
    def publish(self):
        '''
        publish publish the velocity message in vmsg
        '''
        self.pub.publish(self.vmsg)
        
    #def callback(self,msg):
        '''
        callback reads the actuall position of the robot, computes the 
        appropiate velocity, publishes it and check if the goal is reached
        '''

    def scan_callback(self, msg):

        s = self.sensor_spacing

        # Range
        rng = np.array(msg.ranges)
        rng = rng[0:len(msg.ranges):s]

        # Angle
        ang = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ang = ang[0:len(msg.ranges):s]

        # Point features
        points = np.vstack((rng * np.cos(ang),
                            rng * np.sin(ang)))

        # Set Maximum sensing range
        msg.range_max = self.sensor_max_range

        # Acquire points within the maximum range
        points = points[:, rng < msg.range_max]
        
        
        # Split & Merge aLgorithm to get the line features
        lines = splitandmerge(points)

        #print lines

        self.i = self.i + 1
        nss = 'scan_line' + str(self.i)

        utils.publish_lines(lines, self.pub_line, frame=msg.header.frame_id,
                     time=msg.header.stamp, ns=nss, color=(1,0,0), marker_id=1, thickness=0.05)
       
        ###################################################################
        ##### Compute stuff for obstacle avoidance - Daudt's method 2 #####
        ###################################################################

        th_up = 1.5

        # prune points
        ang = ang[rng < msg.range_max]
        rng = rng[rng < msg.range_max]
        if rng.size < 1:
            self.oa_v = 1
            self.oa_w *= 0.8
            return

        # Linear velocity factor
        min_dist = np.min(rng)

        # Angular velocity factor
        w = np.where(rng == min_dist)
        angle = np.mean(ang[w])

        if min_dist < th_up:
            if angle <= 0:
                self.oa_w = 1.0
            else:
                self.oa_w = -1.0

        else:
            self.oa_w *= 0.8

        
    def odom_callback(self, msg):
        '''
        Publishes a tf based on the odometry of the robot.
        '''
        # Translation

        if self.params_loaded == False:
            return
        self.read_position(msg)
        self.compute_velocity()
        self.publish()
        self.check_goal()


        trans = (msg.pose.pose.position.x, 
                 msg.pose.pose.position.y, 
                 msg.pose.pose.position.z)
        
        # Rotation
        rot = (msg.pose.pose.orientation.x,
               msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z,
               msg.pose.pose.orientation.w)
        
        # Publish transform
        self.tfBroad.sendTransform(translation = trans,
                                   rotation = rot, 
                                   time = msg.header.stamp,
                                   parent = '/world',
                                   child = '/base_footprint')

        self.tfBroad.sendTransform(translation = (0,0,0),
                                   rotation = tf.transformations.quaternion_from_euler(0,0,0
                                    ), 
                                   time = msg.header.stamp,
                                   child = '/odom',
                                   parent = '/world')
    def drive(self):
        '''
        drive is a neede function for the ros to run untill somebody stops
        the driver
        '''
        self.print_goal()
        while not rospy.is_shutdown():
            utils.publish_lines(self.map, self.pub_map, frame='world',
                            ns='map', color=(25, 25, 25), thickness=0.2)
            utils.publish_lines(self.trajectory, self.pub_traj, frame='world',
                            ns='trajectory', color=(0, 0, 1), thickness=0.05)
            rospy.sleep(0.03)
        
    def load_goals(self):
        '''
        load_goals loads the goal (or goal list for the option al part) into
        the x y theta variables.
        '''
        

        filepath = rospy.get_param('file',0)
        x_goal = rospy.get_param('x', self.x_start)
        y_goal = rospy.get_param('y', self.y_start)
        
        grid_map = np.array(imread(filepath))
        grid_map = grid_map[:,:,0]
        
        q_start = [self.x_start, self.y_start] #[200,200]
        q_goal =  [x_goal, y_goal] #[200,350]
        
        print q_start
        print q_goal
        
        k = 10000
        delta_q = 10
        p = 0.3

        path = rrt(grid_map,q_start,q_goal,k,delta_q,p)

        print 'Path: ',path
        n = path.shape[0]

        self.x = (path[:,1]-self.offset_y)*self.scale
        self.y = (path[:,0]-self.offset_x)*self.scale
        self.theta = 0*self.x
        for i in np.arange(0,n-1):
            x1 = self.x[i]
            y1 = self.y[i]
            x2 = self.x[i+1]
            y2 = self.y[i+1]
            self.theta[i] = self.angle_wrap(np.arctan2(y2-y1,x2-x1))
            self.trajectory = np.vstack((self.trajectory, np.array([self.x[i], self.y[i], self.x[i+1], self.y[i+1]])));
        
        self.theta[n-1] = self.theta[n-2]
        
        print 'Trajectory: ', self.trajectory

        self.num_goals = self.x.size
        self.params_loaded = True
        self.active_goal = 1
        self.print_goals()
        
    def next_goal(self):
        '''
        next_goal increments the index of the goal in 1 and checks whether
        or not the robot has reached the final goal
        
        TODO modify for the optional part
        '''
        print('Loading next goal')
        self.active_goal = self.active_goal + 1
        if self.active_goal >= self.num_goals:
            # save the robot's last position
            rospy.set_param('cur_pos_x', rospy.get_param('x', self.x_start))
            rospy.set_param('cur_pos_y', rospy.get_param('y', self.y_start))
            rospy.set_param('cur_pos_theta', rospy.get_param('theta', self.theta_start))
            rospy.set_param('cur_pos_world_x', self.position_x)
            rospy.set_param('cur_pos_world_y', self.position_y)
            rospy.set_param('cur_pos_world_theta', self.position_theta)
            rospy.signal_shutdown('Final goal reached!')
            self.active_goal = self.active_goal - 1 # Just in case
        self.print_goal()
        
    def read_position(self,msg):
        '''
        read_position copy the position received in the message (msg) to the
        internal class variables self.position_x, self.position_y and 
        self.position_theta
        
        Tip: in order to convert from quaternion to euler angles give a look
        at tf.transformations
        
        TODO implement!
        '''
        ori = msg.pose.pose.orientation # Extract orientation object
        quat = [ori.x, ori.y, ori.z, ori.w] # Transform into appropriate form
        angles = euler_from_quaternion(quat) # Get angles from quaternion

        # Save position into object variables
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_theta = angles[2]
        
    def compute_velocity(self):
        '''
        compute_velocity computes the velocity which will be published.
        
        '''

        dx = self.x[self.active_goal] - self.position_x # distance in x between turtle and target
        dy = self.y[self.active_goal] - self.position_y # distance in y between turtle and target
        d = np.linalg.norm([dx,dy]) # total distance between turtle and target
        epsilon = 0.0001


        if d > self.goal_th_xy/2: # Using threshold/2 to avoid entering and leaving goal radius
            theta_target = np.arctan2(dy,dx) # angle to target
            dt = self.angle_wrap(theta_target - self.position_theta) # angle difference

            # For derivative controller
            d_deriv = d - self.d_prev
            self.d_prev = d
            dt_deriv = dt - self.dt_prev
            self.dt_prev = dt


            # Calculate control signals
            linv = (self.kp_v*d + self.kd_v*d_deriv)*(np.cos(dt/2)**32) * self.oa_v
            self.vmsg.linear.x = np.min([linv/(np.sqrt(np.abs(linv)) + epsilon),0.5])
            angv = self.kp_w*dt + self.kd_w*dt_deriv + self.oa_w
            self.vmsg.angular.z = angv/(np.sqrt(np.abs(angv)) + epsilon)


        elif not self.has_arrived_ang():
            dt = self.angle_wrap(self.theta[self.active_goal] - self.position_theta) # angle difference

            # For derivative controller
            dt_deriv = dt - self.dt_prev
            self.dt_prev = dt

            # Calculate control signals
            self.vmsg.linear.x = 0
            angv = self.kp_w*dt + self.kd_w*dt_deriv
            self.vmsg.angular.z = angv/np.sqrt(np.abs(angv))


    def angle_wrap(self,ang):
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
