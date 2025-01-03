#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

# Define goal points
GOALS = {
    1: (-6.53114128112793, -2.053840398788452, 0.0, 0.7303135602327445),
    2: (-6.030522346496582, 3.3743555545806885, 0.0, 0.999899903871913),
    3: (-1.9796538352966309, 0.4332816004753113, 0.0, 0.9999923126088655),
    4: (1.0197572708129883, 2.6753461360931396, 0.0, 0.7262415664592414),
    5: (6.066136837005615, 0.7363654971122742, 0.0, 0.7099098463145482),
    6: (6.026930809020996, -1.3034342527389526, 0.0, 0.7161948113421561),
    7: (1.803402066230774, -2.9521355628967285, 0.0, 0.7175668682640408)
}

def send_goal(goal_id):
    if goal_id not in GOALS:
        rospy.logwarn("Invalid goal ID. Please choose a valid goal number.")
        return

    x, y, z, w = GOALS[goal_id]
    
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('goal_sender', anonymous=True)
    rospy.sleep(1)  # Give time for the publisher to establish connection
    
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    goal.pose.orientation.w = w
    
    rospy.loginfo(f"Sending goal {goal_id}: x={x}, y={y}, z={z}, w={w}")
    pub.publish(goal)
    rospy.loginfo("Goal sent successfully!")

if __name__ == '__main__':
    try:
        goal_id = int(input("Enter goal number (1-7): "))
        send_goal(goal_id)
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        rospy.logwarn("Invalid input. Please enter a number between 1 and 7.")

