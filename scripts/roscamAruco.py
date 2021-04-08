#!/usr/bin/env python

import rospy
import numpy as np
#from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray as array64
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CompressedImage as CsensImg
#from sensor_msgs.msg import PointCloud2 as sensPCld
import cv2 as cv
#from cv2 import aruco as aruco
import cv2.aruco as aruco
from roscamLibrary import singleAruRelPos


#------------------------------------------------
#NOTE:
#   tVect       manipulatore
#    X             -Z
#    Y              Y
#    Z              X
#matrice rotazione da manipulatore a camera:
#
#        Ry(pi/2)= 0 0 1
#                  0 1 0
#                 -1 0 0
#PUBLISHER DI ARRAY:
#aruco_position_pub = rospy.Publisher('/almax/aruco_target',Float64MultiArray,queue_size=20)
#array = [69.1,0,1,33,1,1,1,0]
#robaccia = Float64MultiArray(data=array)
#aruco_position_pub.publish(robaccia)
#------------------------------------------------

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

cameraMatr = np.matrix([[462.1379497504639, 0.0, 320.5],\
                                 [0.0, 462.1379497504639, 240.5],\
                                 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
cameraDistCoefs = np.array([1e-08, 1e-08, 1e-08, 1e-08, 1e-08])

cameraFocLen= 462.137

#targetImg=cv.imread('test-imgs/moriginal/m582orig.jpg')
#_,targetId,_=aruco.detectMarkers(targetImg,ARUCO_DICT,parameters=ARUCO_PARAMETERS)#targetId=582

arucoObjectDict={'cube5s':(582,#id
                            40),#size [mm]
#                'cube5d:(582,
#                          40),
#                'cube10d':(582,
#                            90),
                'centrifugaBase': (273,
                                   100),
                'centrifugaAxial': (429,
                                    100), 
                'centrifugaTangent': (221,
                                      100)               
                } 

targetObject='cube5s'
targetId,targetSize=arucoObjectDict[targetObject]


from cv_bridge import CvBridge
bridge=CvBridge()

def arTargetTalker(mymode,myArData1,myArdata2):
        
    if mymode=='float64':
#        pub = rospy.Publisher('codio', array64, queue_size=1)        
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
#        pub = rospy.Publisher('codio', Pose, queue_size=1)
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



 
def callbackRaw(raw_img):
#    
    #imgDim=(width,heigth)=(data.height, data.width)
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
#    cv_imageSrc=cv_image.copy()
#    cv.imshow("raw image", cv_image)
#    cv.waitKey(15)
#    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
#    for i, corner in zip(ids, corners):
#        print("Found aruco with ID: {}; Corners: {}".format(i[0], corner[0]))
    detCorners, detIds, _ = aruco.detectMarkers(cv_image, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    if detIds is not None and len(detIds) >= 1: # Check if at least one marker has been found

        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
            
        for mId, aruPoints in zip(detIds, detCorners):
                
            detAruImg,aruDistnc,Pmatr,rVecs=singleAruRelPos(detAruImg,aruPoints,mId,targetSize,
                                          cameraMatr,cameraDistCoefs,cameraFocLen,superimpAru='distance')
            rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
    #            if mId==targetId:
    #                msgmode='float64'
    ##                try:
    #                arTargetTalker(msgmode,rotMatr,tVect)
#                except rospy.ROSInterruptException:
#                    pass
#            print("rotMatr: ",rotMatr)
#           print("tVect: ",tVect)
#           print('marker',mId, "has distance:",aruDistnc)
#           cv.imshow('detmarkersected ',detAruImg)
    else:
#        print("no marker detected")   
        detAruImg=cv_image.copy()
#   green contourning of found markers
#    detectedArucoImg = aruco.drawDetectedMarkers(cv_image, corners, borderColor=(0, 255, 0))
    
    #red contourning of rejected markers
#    if rejectedImgPoints is not None and len(rejectedImgPoints) >= 1:
#        detectedArucoImg = aruco.drawDetectedMarkers(detectedArucoImg, rejectedImgPoints, borderColor=(0, 0, 255))
#    cv.imshow("raw image", cv_image)
    cv.imshow('videoFeed',detAruImg)
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
    
def listener(myCam,myTop,myType,myCallk):    
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
    listener(myCamera,myTopicFull[0],myTopicFull[1],myTopicFull[2])



#bibliography
#     (1)
#compressed images
#cast in np array and cv2.imdecode
#http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
#    (2)
#https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
#   (3)
#https://strawlab.github.io/python-pcl/