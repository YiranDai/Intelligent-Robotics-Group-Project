#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#moving = True
pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
base_data = Twist()



def move():
    rospy.init_node('Mover', anonymous=True)
    rospy.Subscriber("base_scan",LaserScan , callback)
    rospy.spin()

def callback(data):
    #global moving
    base_data.linear.x = 0.0
    left = (-1, sum(data.ranges[:100]), 0)
    upper_left = (-0.5, sum(data.ranges[100:200]), 1)
    forward = (0, sum(data.ranges[200:300]), 2)
    upper_right = (0.5, sum(data.ranges[300:400]), 3)
    right = (1, sum(data.ranges[400:]), 4)
    directions = [left, upper_left, forward, upper_right, right]
    direction = max(directions, key=lambda x: x[1])

    narrowed_range = direction[2] * 100
    angular = data.ranges[narrowed_range : narrowed_range+100]

    base_data.angular.z = direction[0]

    if min(angular[80:]) < 1.5 and min(angular[80:]) < 1.5:
        base_data.angular.z = 10
    elif min(angular[80:]) < 1 :
        base_data.angular.z += -0.6
    elif min(angular[:20]) < 1:
        base_data.angular.z += 0.6
    else:
        base_data.linear.x = 0.1

    '''
    if min(angular[80:]) < 1 :
        base_data.angular.z += -0.2
        if moving:
            moving = False
        else:
            base_data.angular.z = 60
    elif min(angular[:20]) < 1:
        base_data.angular.z += 0.2
        if moving:
            moving = False
        else:
            base_data.angular.z = 60
    else:
        base_data.linear.x = 0.2
        if moving == False:
            moving = True
            '''

    pub.publish(base_data)



if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
