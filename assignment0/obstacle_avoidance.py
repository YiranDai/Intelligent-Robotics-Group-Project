#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def move():

    # rate = rospy.Rate(10) #10hz

    rospy.Subscriber("base_scan",LaserScan , callback)


def callback(data):
  #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data


   angularZ = get_direction(data)


   pub = rospy.Publisher('cmd_vel',Twist, queue_size=100)
   rospy.init_node('Mover', anonymous=True)
   base_data = Twist()
   base_data.linear.x = 0.1
   base_data.angular.z = 0.1
   pub.publish(base_data)


def get_direction(data):

    # value at 0 degrees

    left = data.ranges[0]
    # value at 90 degrees
    front= data.ranges[len(data.ranges)/2]
    # value at 180 degrees
    right= data.ranges[len(data.ranges)-1]




if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
