#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty
import math

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
	pub_1 = rospy.Publisher('/rrbot/joint_base_position_controller/command',Float64,queue_size=10)	
	pub_b = rospy.Publisher('/rrbot/joint_base_mid_position_controller/command',Float64,queue_size=10)
	pub_m = rospy.Publisher('/rrbot/joint_mid_top_position_controller/command',Float64,queue_size=10)
	pub_l = rospy.Publisher('/rrbot/left_gripper_joint_position_controller/command',Float64,queue_size=10)
	pub_r = rospy.Publisher('/rrbot/right_gripper_joint_position_controller/command',Float64,queue_size=10)
	rate = rospy.Rate(50)

	while not rospy.is_shutdown():
	        x = input('enter x = ')
		z = input('enter z = ')
		
		cos_t2 = (x*x + z*z - 2)/2
		sen_t2 = math.sqrt(1 - cos_t2*cos_t2)
		if cos_t2 == 0:
 		 	q2 = 3.14/2
		else:
			q2 = math.atan(sen_t2/cos_t2)

		if x == 0: 
			qq = 3.14/2
		else:
			qq = math.atan(z/x)

		if (1+cos_t2)==0:
			qqq = 3.14/2
		else:
			qqq = math.atan(sen_t2/(1 + cos_t2))

		q1 = abs(qq - qqq)

		q1 = -q1/(3.14/2) + 1
		q2 = -q2*2/3.14
		pub_b.publish(q1)
		pub_m.publish(q2)
		
		print('enviado = ' + str(q1) + ' y ' + str(q2))

		    
		       
 
		rate.sleep()



if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	main()
