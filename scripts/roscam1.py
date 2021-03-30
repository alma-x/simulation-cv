#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String
from PIL import Image
import matplotlib
import numpy as np
from sensor_msgs.msg import Image as simg
#import time

def callback(data):
    heigth,width=data.height, data.width
    dim=(width,heigth)
    raw_img=data.data


    img_array = np.asarray(bytearray(raw_img), dtype=np.uint8)
    cv2.resize(np.uint8(img_array),(width,heigth))
    cv2.imshow("image", img_array)
    cv2.waitKey()
    cv2.destroyAllWindows()
    #print(dati)
    #while True:
	#True
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
