#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#import cv2 as cv
#from matplotlib import pyplot as plt
#import numpy as numpy
from sensor_msgs.msg import Image as simg
#import time

def callback(data):
    heigth,width=data.height, data.width
    dati=data.data
    #print(width,heigth)
    print(dati)
    while True:
	True
    #time.sleep(1)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('camera_listener', anonymous=True)

    rospy.Subscriber("/camera_image/color/image_raw",simg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print('starting')
    listener()
