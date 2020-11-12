from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

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

        mean, sd = initialpose.pose.pose.position.x , np.std(initialpose.pose.covariance)
        self.gaussian = np.random.normal(loc=mean, scale=sd, size=(3,1))
        rospy.loginfo('-----------------')
        rospy.loginfo('covariance')
        rospy.loginfo(initialpose.pose.covariance)
        rospy.loginfo('-----------------')
        rospy.loginfo('gaussian')
        rospy.loginfo(self.gaussian)
        rospy.loginfo('-----------------')
        noise = 6
        initialOrientation = initialpose.pose.pose.orientation
        sdOri = np.pi/4

        poses = self.particlecloud

        for i in range(1):
            nd = np.random.normal(mean, sd,2)
            ndOri = np.random.normal(mean, sdOri)
            pose = Pose()
            #yaw = getHeading(initialOrientation) * orientationNoise
            pose.orientation = rotateQuaternion(initialOrientation, ndOri)
            pose.position.x = initialpose.pose.pose.position.x
            pose.position.y = initialpose.pose.pose.position.y
            pose.position.z = initialpose.pose.pose.position.z
            poses.poses.append(pose)

        return poses


    def update_particle_cloud(self, scan):

        mean, sd = self.estimatedpose.pose.pose.position.x , np.std(self.estimatedpose.pose.covariance)
        self.gaussian = np.random.normal(loc=mean, scale=sd, size=(3,1))

        rospy.loginfo('-----------------')
        rospy.loginfo('covariance')
        rospy.loginfo(self.estimatedpose.pose.covariance)
        rospy.loginfo('-----------------')
        rospy.loginfo('gaussian')
        rospy.loginfo(self.gaussian)
        rospy.loginfo('-----------------')
        '''

        '''

        pass

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
        pass
