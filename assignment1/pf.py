from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
#from random import random
import random
from time import time

import numpy as np

def most_frequent(List):
    counter = 0
    num = List[0]

    for i in List:
        curr_frequency = List.count(i)
        if(curr_frequency > counter):
            counter = curr_frequency
            num = i

    return num, counter


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
        '''
        rospy.loginfo(type(initialpose))
        '''

        self.NUMBER_PREDICTED_READINGS = 200

        rospy.loginfo(initialpose)
        sd = np.std(initialpose.pose.covariance)
        mean = 0
        noise = 6
        initialOrientation = initialpose.pose.pose.orientation
        sdOri = np.pi/4
        # orientation, covariance, noise
        poses = PoseArray()
        for i in range(self.NUMBER_PREDICTED_READINGS):
            nd = np.random.normal(mean, sd,2)
            ndOri = np.random.normal(mean, sdOri)
            pose = Pose()
            #yaw = getHeading(initialOrientation) * orientationNoise
            pose.orientation = rotateQuaternion(initialOrientation, ndOri) #getHeading(q)
            pose.position.x = initialpose.pose.pose.position.x + nd[0] * noise
            pose.position.y = initialpose.pose.pose.position.y + nd[1] * noise
            pose.position.z = initialpose.pose.pose.position.z
            poses.poses.append(pose)

        rospy.loginfo("-----------------------poses----------------")
        rospy.loginfo(poses)
        rospy.loginfo(len(poses.poses))
        rospy.loginfo("-----------------------endposes----------------")


        return poses


    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        rospy.loginfo("out-dated particle cloud")
        #rospy.loginfo(self.particlecloud.poses)
         # update particle filter based on sensor model
        poses = self.particlecloud.poses
        for pose in poses:
            rospy.loginfo(pose)
            rospy.loginfo('-------------------------------')
        # 1- assign w self.sensor_model.get_weight()
        rospy.loginfo(len(poses))
        weights = []
        for i in range(len(poses)):
            weights += [self.sensor_model.get_weight(scan, poses[i])]

        rospy.loginfo((weights))
        normWeights = [float(i)/sum(weights) for i in weights]
        # 2- resample particles
        resampled=PoseArray()
        c=[normWeights[0]]
        for i in range(1,len(normWeights)):
            c += [c[i-1]+normWeights[i]]
        u = random.uniform(0.0, 1.0/len(normWeights))
        rospy.loginfo('c: ')
        rospy.loginfo(c)
        rospy.loginfo('u: ')
        rospy.loginfo(u)
        i = 0
        for j in range(len(normWeights)):
            while u > c[i] :
                i+=1
                rospy.loginfo('u > c[i]')
            resampled.poses.append(poses[i])
            u += 1.0/len(normWeights)
            rospy.loginfo('u: ')
            rospy.loginfo(u)

        # 3- self.particlecloud = poses
        rospy.loginfo('------------------------updated poses------------------')
        self.particlecloud = resampled
        for pose in resampled.poses:
            rospy.loginfo(pose)
            rospy.loginfo('-------------------------------')
        #rospy.loginfo(self.particlecloud.poses)
        #rospy.loginfo('-------------------end updated------------')





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


        rospy.loginfo('--------------------estimatedpose-------------')
        rospy.loginfo(most_frequent(self.particlecloud.poses))
        rospy.loginfo('--------------------end estimatedpose-------------')


        pose, frequent = most_frequent(self.particlecloud.poses)

        #for kidnapping, update particlecloud

        if(frequent > len(self.particlecloud.poses)/2):
            close =0.5
            far = 2
            lenPoses = len(self.particlecloud.poses)
            self.particlecloud.poses[0] = pose
            for i in range(1, lenPoses):
                temp = pose

                if i < lenPoses/2:
                    temp.position.x = pose.position.x + random.uniform(-close,close)
                    temp.position.y = pose.position.y + random.uniform(-close, close)

                else:
                    temp.position.x = pose.position.x + random.uniform(-far,far)
                    temp.position.y = pose.position.y + random.uniform(-far, far)
                self.particlecloud.poses[i] = temp


        return pose

