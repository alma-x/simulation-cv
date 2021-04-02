#!/usr/bin/env python
import cv2 as cv
import rospy
#from std_msgs.msg import String
#from PIL import Image
#import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CompressedImage as CsensImg
#frofm sensor_msgs.msg import PointCloud2 as sensPCld

from cv import aruco as aruco

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)


from cv_bridge import CvBridge
bridge=CvBridge()
 
def callbackRaw(raw_img):
    #imgDim=(width,heigth)=(data.height, data.width)
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_imageSrc=cv_image.copy()
#    cv.imshow("raw image", cv_image)
#    cv.waitKey(15)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
#    for i, corner in zip(ids, corners):
#        print("Found aruco with ID: {}; Corners: {}".format(i[0], corner[0]))
    
#   green contourning of found markers
    detectedArucoImg = aruco.drawDetectedMarkers(cv_image, corners, borderColor=(0, 255, 0))
    
    #red contourning of rejected markers
#    if rejectedImgPoints is not None and len(rejectedImgPoints) >= 1:
#        detectedArucoImg = aruco.drawDetectedMarkers(detectedArucoImg, rejectedImgPoints, borderColor=(0, 0, 255))
    cv.imshow("raw image", cv_image)
    cv.waitKey(15)
 
def callbackCompr(cmpr_img):#(1)
    imgarr = np.fromstring(cmpr_img.data, np.uint8)
    # could possibly try also direct array recast, but already working
    cv_image=cv.imdecode(imgarr,cv.IMREAD_UNCHANGED)
    print('compressed dimension')
    print(cv_image.shape)
    cv.imshow("compressed image", cv_image)
    cv.waitKey(15)

#def callbackComprDep(cmpr_img):TODO#(1)
#    imgarr = np.fromstring(cmpr_img.data, np.uint8)
#    cv_image=cv.imdecode(imgarr,cv.IMREAD_UNCHANGED)
#    cv.imshow("image", cv_image)
#    cv.waitKey(15)
    
#def callbackPCld(pcld_img):#(3)
##    no pcl_ros method in python, which worked smootlhy
##    alternative is python_pcl
##    cv_image=
#    cv.imshow("point cloud image", cv_image)
#    cv.waitKey(15)
    
def listener(myCam,(myTop,myType,myCallk)):
    rospy.init_node('camera_listener', anonymous=True)
    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
#    rospy.Subscriber("/camera_image_fix/color/image_raw/compressed",CsensImg,callbackCompr,queue_size = 1)
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
#                'raw depth':("/color/image_raw/compressedDepth",
#                             CsensImg,
#                             callbackComprDep),
#                'point cloud':("/depth/color/points",
#                               sensPCld,
#                               callbackPCld),
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
