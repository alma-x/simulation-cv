#!/usr/bin/env python
import cv2 as cv
import rospy
#from std_msgs.msg import String
#from PIL import Image
#import matplotlib.pyplot as plt
#import numpy as np
from sensor_msgs.msg import Image as sensImg
from cv_bridge import CvBridge
#import time
bridge=CvBridge()
 
def callback(raw_img):
    #heigth,width=data.height, data.width
    #dim=(width,heigth)
    #print(dim)
    #raw_img=data
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv.waitKey(30)
    cv.imshow("image", cv_image)
    
    
def listener():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('camera_listener', anonymous=True)
 
    rospy.Subscriber("/camera_image_fix/color/image_raw",sensImg, callback,queue_size = 1)
    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Closing')
    cv.destroyAllWindows() 
if __name__ == '__main__':
    print('starting')
    listener()
 


