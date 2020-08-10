#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty

base = 0
mid = 0
gripper = 0

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
	rospy.init_node('arm_teleop')
	pub_b = rospy.Publisher('/rrbot/joint_base_mid_position_controller/command',Float64,queue_size=10)
	pub_m = rospy.Publisher('/rrbot/joint_mid_top_position_controller/command',Float64,queue_size=10)
	pub_l = rospy.Publisher('/rrbot/left_gripper_joint_position_controller/command',Float64,queue_size=10)
	pub_r = rospy.Publisher('/rrbot/right_gripper_joint_position_controller/command',Float64,queue_size=10)
	rate = rospy.Rate(50)

	while not rospy.is_shutdown():
	    base = input('enter your angle = ')
            mid = input('enter your angle = ')
            gripper = input('gripper here = ')

	    pub_b.publish(base/90)
            pub_m.publish(mid/90)
	    pub_l.publish(gripper)
	    pub_r.publish(-gripper)
            rospy.loginfo('publicando')

            
                

            rate.sleep()



if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	main()
