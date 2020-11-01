from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):

    def _init_(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self)._init_()

        # ----- Set motion model parameters

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
        rospy.loginfo(type(initialpose))
        rospy.loginfo(initialpose)

        # orientation, covariance, noise
        poses = PoseArray()
        for i in range(self.NUMBER_PREDICTED_READINGS):
            pose = Pose()
            pose.orientation = initialpose.pose.pose.orientation
            pose.position.x = initialpose.pose.pose.position.x + random()
            pose.position.y = initialpose.pose.pose.position.y + random()
            pose.position.z = initialpose.pose.pose.position.z
            poses.poses.append(pose)

        rospy.loginfo(poses)
        return poses


    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        rose.loginfo("update particle cloud")
         # idk, update particle filter based on sensor model
        poses = self.particlecloud
        # 1- assign w self.sensor_model.get_weight()
        weights = []
        for pose in poses:
            weights.append(self.sensor_model.get_weight(self,scan,pose))

        # 2- resample particles
        resampled=[]
        c=[weights[0]]
        for i in range(1,len(weights)):
            c[i]= c[i-1]+weights[i]
        u = random.random(0,1/len(weights))

        i = 0
        for j in range(len(weights)):
            while u > c[i]:
                i+=i
            resampled.append(poses[i])
            u += 1/len(weights)

        # 3- self.particlecloud = poses
        self.particlecloud = resampled
        rospy.loginfo(self.particlecloud)
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
        rospy.loginfo(most_frequent(self.particlecloud))
        return most_frequent(self.particlecloud)


    def most_frequent(List):
        counter = 0
        num = List[0]

        for i in List:
            curr_frequency = List.count(i)
            if(curr_frequency> counter):
                counter = curr_frequency
                num = i

        return num
