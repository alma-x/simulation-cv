#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray as array64
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Pose
#from geometry_msgs import Pose
#import numpy as np


#PUBLISHER DI ARRAY:
#aruco_position_pub = rospy.Publisher('/almax/aruco_target',Float64MultiArray,queue_size=20)
#array = [69.1,0,1,33,1,1,1,0]
#robaccia = Float64MultiArray(data=array)
#aruco_position_pub.publish(robaccia)
#------------------------------------------------


def talker(mymode):
        
    if mymode=='float64':
        pub = rospy.Publisher('codio', array64, queue_size=1)        
        #    mydata=([[0, 0, 1, 250],[0, 1, 0, 15],[-1, 0, 0, -21]])
        mydata=([0, 0, 1, 250,0, 1, 0, 15,-1, 0, 0, -21])
        mymsg = array64()
    #    mymsg.data=mydata.reshape([12])
        mymsg.data=mydata
    #    mymsg.data = ([1.0, 2.0],[3.0, 4.0])
        mymsg.layout.data_offset = 0 
        mymsg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        # dim[1] is the vertical dimension of your matrix
        mymsg.layout.dim[0].label = "row"
        mymsg.layout.dim[0].size = 3
        mymsg.layout.dim[0].stride = 12
        # dim[1] is the horizontal dimension of your matrix
        mymsg.layout.dim[1].label = "col"
        mymsg.layout.dim[1].size = 4
        mymsg.layout.dim[1].stride = 4
    else:
        pub = rospy.Publisher('codio', Pose, queue_size=1)
        mymsg=Pose()
        mymsg.position.x=1
        mymsg.position.y=1
        mymsg.position.z=1
        mymsg.orientation.x=1
        mymsg.orientation.y=1
        mymsg.orientation.z=1
        mymsg.orientation.w=1
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        pub.publish(mymsg)
        rate.sleep()

if __name__ == '__main__':
    msgmode='float64'
    try:
        talker(msgmode)
    except rospy.ROSInterruptException:
        pass
    





