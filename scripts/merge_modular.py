#!/usr/bin/env python

import rospy
import sys
import numpy as np

from geometry_msgs.msg import Twist


dirMat = np.matrix('-1 -1;-1 0;-1 1;0 -1;0 0;0 1;1 -1;1 0;1 1')
magMat = np.matrix('2;3;1;2;0;1;3;1.5;1.5')
constVel = 0.1
pubs = []

def pubs_setup():
    global pubs
    
    pub1 = rospy.Publisher("/mod1/cmd_vel", Twist, queue_size=10)
    pub2 = rospy.Publisher("/mod2/cmd_vel", Twist, queue_size=10)
    pub3 = rospy.Publisher("/mod3/cmd_vel", Twist, queue_size=10)
    pub4 = rospy.Publisher("/mod4/cmd_vel", Twist, queue_size=10)
    pub5 = rospy.Publisher("/mod5/cmd_vel", Twist, queue_size=10)
    pub6 = rospy.Publisher("/mod6/cmd_vel", Twist, queue_size=10)
    pub7 = rospy.Publisher("/mod7/cmd_vel", Twist, queue_size=10)
    pub8 = rospy.Publisher("/mod8/cmd_vel", Twist, queue_size=10)
    pub9 = rospy.Publisher("/mod9/cmd_vel", Twist, queue_size=10)
    pubs.append(pub1)
    pubs.append(pub2)
    pubs.append(pub3)
    pubs.append(pub4)
    pubs.append(pub5)
    pubs.append(pub6)
    pubs.append(pub7)
    pubs.append(pub8)
    pubs.append(pub9)

def model_pub():
    global pubs, dirMat, constVel, magMat

    msg = Twist()
    
    for i in range(0, 4):
        cmdVelX = constVel * dirMat[i, 0] * (magMat[i]-0.416)
        cmdVelY = constVel * dirMat[i, 1] * (magMat[i]-0.226)
        msg = Twist()
        msg.linear.x = cmdVelX
        msg.linear.y = cmdVelY
        pubs[i].publish(msg)
    for i in range(5, 9):
        cmdVelX = constVel * dirMat[i, 0] * (magMat[i]-0.416)
        cmdVelY = constVel * dirMat[i, 1] * (magMat[i]-0.226)
        msg = Twist()
        msg.linear.x = cmdVelX
        msg.linear.y = cmdVelY
        pubs[i].publish(msg)

if __name__ == '__main__':
    rospy.init_node('simple_merger', anonymous=True)
    rate = rospy.Rate(25)
    pubs_setup()

    while not rospy.is_shutdown():
        try:
            model_pub()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)
