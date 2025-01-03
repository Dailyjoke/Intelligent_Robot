#!/usr/bin/env python
from std_msgs.msg import String
import rospy

rospy.init_node('python_hello_node')
while not rospy.is_shutdown():
    rospy.loginfo('Hello World')
    rospy.sleep(1)