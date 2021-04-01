#!/usr/bin/env python
import cv2 as cv
import rospy
#from std_msgs.msg import String
#from PIL import Image
#import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CompressedImage as CsensImg
#from sensor_msgs.msg import PointCloud2 as sensPCld

from cv_bridge import CvBridge
bridge=CvBridge()
 
def callbackRaw(raw_img):
    #imgDim=(width,heigth)=(data.height, data.width)
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    #cv.waitKey(15)#???
    cv.imshow("raw image", cv_image)
    cv.waitKey(15)    
 
def callbackCompr(cmpr_img):#(1)
    imgarr = np.fromstring(cmpr_img.data, np.uint8)
    # could possibly try also direct array recast, but already working
    cv_image=cv.imdecode(imgarr,cv.IMREAD_UNCHANGED)
    cv.imshow("compressed image", cv_image)
    cv.waitKey(15)

#def callbackComprDep(cmpr_img):TODO#(1)
#    imgarr = np.fromstring(cmpr_img.data, np.uint8)
#    cv_image=cv.imdecode(imgarr,cv.IMREAD_UNCHANGED)
#    cv.imshow("image", cv_image)
    cv.waitKey(15)
    
#def callbackPCld(pcld_img):#(3)
##    no pcl_ros method in python, which worked smootlhy
##    alternative is python_pcl
##    cv_image=
#    cv.imshow("point cloud image", cv_image)
#    cv.waitKey(15)

def listener(myCam,(myTop,myType,myCallk)):
    rospy.init_node('camera_listener', anonymous=True)
    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)

    try:
        rospy.spin()
    except KeyboardInterrupt:#what about adding waitKey() here?
        print('Closing')
    cv.destroyAllWindows()
    

camDict={'moving':"/camera_image",
            'fixed':"/camera_image_fix"}

topicDict={'raw compressed':("/color/image_raw/compressed",
                             CsensImg,
                             callbackCompr),
#                'raw depth':(imageTopic="/color/image_raw/compressedDepth",
#                             imageType=CsensImg,
#                             imageCallback=callbackComprDep),
#                'point cloud':(imageTopic="/depth/color/points",
#                               imageType=sensPCld,
#                               imageCallback=callbackPCld),
            'raw depth': ("/depth/image_rect_raw",
                    sensImg,
                    callbackRaw),
            'raw':("/color/image_raw",
                    sensImg,
                    callbackRaw)    
            }   

if __name__ == '__main__':
    myCamera=camDict['moving']
    myTopicFull=topicDict['raw']
    print('connecting to:'+myCamera+myTopicFull[0]+'...')
    listener(myCamera,myTopicFull)
 

#bibliography
#     (1)
#compressed images
#cast in np array and cv2.imdecode
#http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
#    (2)
#https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
#   (3)
#https://strawlab.github.io/python-pcl/