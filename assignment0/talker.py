#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def talker():

	rospy.Subscriber("base_scan",LaserScan , callback)

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
	rospy.init_node('Mover', anonymous=True)
	# rate = rospy.Rate(10) #10hz
	while not rospy.is_shutdown():
		base_data=Twist()
		base_data.linear.x = -0.2
		base_data.angular.z = 0.2
		pub.publish(base_data)
		# rate.sleep()

def callback(data):
  #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data

  print((data.ranges[0]))

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
