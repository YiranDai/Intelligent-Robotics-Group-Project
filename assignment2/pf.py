from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from pf_base import PFLocaliserBase
import math
import rospy
from geometry_msgs.msg import Twist

from util import rotateQuaternion, getHeading
from random import random

from time import time

import numpy as np

class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.1 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.1 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.1 # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.sd = 0.0

    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.

        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        #Initial belief is normally distributed
        self.estimatedpose = initialpose

        mean = initialpose.pose.pose.position.x
        self.sd = np.std(initialpose.pose.covariance)
        self.gaussian = np.random.normal(loc=mean, scale=self.sd, size=(3,1))

        rospy.loginfo('-----------------')
        rospy.loginfo('sd: ')
        rospy.loginfo(self.sd)
        rospy.loginfo('-----------------')
        '''
        rospy.loginfo('gaussian')
        rospy.loginfo(self.gaussian)
        rospy.loginfo('-----------------')
        noise = 6
        '''
        initialOrientation = initialpose.pose.pose.orientation
        sdOri = np.pi/4

        poses = self.particlecloud

        for i in range(1):
            nd = np.random.normal(mean, self.sd,2)
            ndOri = np.random.normal(mean, sdOri)
            pose = Pose()
            #yaw = getHeading(initialOrientation) * orientationNoise
            pose.orientation = initialOrientation
            pose.position.x = initialpose.pose.pose.position.x
            pose.position.y = initialpose.pose.pose.position.y
            #pose.position.z = initialpose.pose.pose.position.z
            poses.poses.append(pose)

        return poses


    def update_particle_cloud(self, scan):

        '''
        rospy.loginfo(self.particlecloud.poses)
        rospy.loginfo('------------------------')
        rospy.loginfo(self.estimatedpose)
        '''

        '''
        self.estimatedpose.pose.pose.position.x = self.particlecloud.poses[0].position.x
        self.estimatedpose.pose.pose.position.y = self.particlecloud.poses[0].position.y
        self.estimatedpose.pose.pose.orientation = self.particlecloud.poses[0].orientation
        '''
        #self.estimatedpose.pose = self.particlecloud.poses[0]

        '''
        self.estimatedpose.pose.pose.position = self.particlecloud[0].poses.position
        '''
        '''
        mean, sd = self.estimatedpose.pose.pose.position.x , np.std(self.estimatedpose.pose.covariance) + self.particlecloud[0].position.z
        self.gaussian = np.random.normal(loc=mean, scale=sd, size=(3,1))
        '''
        #rospy.loginfo(self.estimatedpose)
        rospy.loginfo('-----------------')
        rospy.loginfo('pose')
        rospy.loginfo(self.particlecloud.poses[0])


        rospy.loginfo('-----------------')
        rospy.loginfo('sd: ')
        rospy.loginfo(self.sd)
        ox = self.particlecloud.poses[0].position.x
        oy = self.particlecloud.poses[0].position.y
        #oa = getHeading(self.particlecloud.poses[0].orientation) * (180/np.pi)
        #estimated_range = self.sensor_model.calc_map_range(ox, oy, oa)
        rospy.loginfo(self.particlecloud.poses[0])
        w = self.sensor_model.get_weight(scan, self.particlecloud.poses[0])
        if w < 1.1 and w > 0.9:
            w +=0.3
        self.sd += abs((self.estimatedpose.pose.pose.position.x - ox) + (self.estimatedpose.pose.pose.position.y - oy))/2

        self.sd = self.sd / w

        if( self.sd >3):
            self.sd = 3
        elif (self.sd < 0.5):
            self.sd = 0.5

# sd = 8, w = 1 -> 1/8 = 0.12   sd = 0.12 w 1 -> 1/0.12
        rospy.loginfo('-----------------')
        rospy.loginfo('sd: ')
        rospy.loginfo(self.sd)
        rospy.loginfo('-----------------')
        rospy.loginfo('weight')
        rospy.loginfo(w)
        rospy.loginfo('-----------------')

        newX = np.random.normal(ox, self.sd)
        newY = np.random.normal(oy, self.sd)
        tempPose = self.particlecloud.poses[0]
        tempPose.position.x = newX
        tempPose.position.y = newY
        newW = self.sensor_model.get_weight(scan, tempPose)
        w = w / (newW + w)
        newW = 1 - w
        rospy.loginfo('before pose')
        rospy.loginfo(tempPose)
        rospy.loginfo('-----------------')

        self.particlecloud.poses[0].position.x = float(w * ox + newW * newX)
        self.particlecloud.poses[0].position.y =  w * oy + newW * newY
        rospy.loginfo('final pose')
        rospy.loginfo(self.particlecloud.poses[0])
        rospy.loginfo('-----------------')


    def estimate_pose(self):


        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).

        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.

        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.

         """
        return self.particlecloud.poses[0]

