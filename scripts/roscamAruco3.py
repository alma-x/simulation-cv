#!/usr/bin/env python3

import rospy
import numpy as np

#from std_msgs.msg import String
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CompressedImage as CsensImg
#frofm sensor_msgs.msg import PointCloud2 as sensPCld
from ur3_control.srv import aruco_service,aruco_serviceResponse

import cv2 as cv
import cv2.aruco as aruco
#from cv2 import aruco as aruco
from cv_bridge import CvBridge
from roscamLibrary3 import singleAruRelPos as singleAruRelPos

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
                        [0.0000e+00, 0.0000e+00, 1.0000e+00]])
cameraDistCoefs = np.array([1e-08, 1e-08, 1e-08, 1e-08, 1e-08])

cameraFocLen= 462.137

#targetImg=cv.imread('test-imgs/moriginal/m582orig.jpg')
#_,targetId,_=aruco.detectMarkers(targetImg,ARUCO_DICT,parameters=ARUCO_PARAMETERS)#targetId=582

aruco_success=False
msgVector=[0,0,0]#np.zeros([numRows,numCols])
msgRotMatrix=[[0,0,0],[0,0,0],[0,0,0]]#should this be a matrix?

aruTargetDict={'cube5s':(582,
                        40),
#                'cube5d':(582,
#                        40),
#                'cube10d':(582,
#                        90),
#                'centrifugaBase':(273,
#                        100),
#                  'centrifugaAxial':(429,
#                        100),
#                'centrifugaTangential':(221,
#                        100)
#                'panelSwitch':(,
#                        )
#                'newWord':(wordArucoId,
#                        wordMarketSize)
                }
           
(targetMarkId,targetMarkSize)=targetMarker=aruTargetDict['cube5s']
#may introduce external request for this

bridge=CvBridge()
 
def callbackRaw(raw_img):
    global aruco_success
    global msgVector
    global msgRotMatrix
#    cv.imshow("raw image", cv_image)
    #imgDim=(width,heigth)=(data.height, data.width)
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')

    detCorners, detIds, _ = aruco.detectMarkers(cv_image, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    if detIds is not None and len(detIds) >= 1: # Check if at least one marker has been found
        
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
            
        for mId, aruPoints in zip(detIds, detCorners):
                
            detAruImg,aruDistnc,Pmatr,rVecs=singleAruRelPos(detAruImg,aruPoints,mId,targetMarkSize,
                                          cameraMatr,cameraDistCoefs,cameraFocLen,superimpAru='distance')
            rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
            msgRotMatrix=rotMatr
#            print("rotMatr: ",rotMatr)
            print("tVect: ",tVect)
#            print('marker',mId, "has distance:",aruDistnc)
            if mId==targetMarkId:
                aruco_success=True
                msgVector=tVect
            else:
                aruco_success=False
    else:
#        print("no marker detected")
        aruco_success=False
        detAruImg=cv_image.copy()#

    cv.imshow('video feed',detAruImg)
    cv.waitKey(15)
    
toggleWristLengthRecovery=0

def callback_service(req):
    global aruco_success
    global msgVector
#    if aruco_success: print("ARUCO SUCCESS:TRUE")
    return aruco_serviceResponse(
            success=aruco_success,
            x=0.001*msgVector[2] +(0.08 if toggleWristLengthRecovery else 0),#[m]
            y=0.001*msgVector[0],   
            z=0.001*msgVector[1]
#vector??
vector=[msgRotMatrix[0,0],msgRotMatrix[0,1],msgRotMatrix[0,2],msgRotMatrix[1,0],msgRotMatrix[1,1],msgRotMatrix[1,2],msgRotMatrix[2,0],msgRotMatrix[2,1],msgRotMatrix[2,2]]
            )
    
#------------------------------------------------
#NOTE:
#   tVect       tool0/maniulator e.e reference frame
#    X              Z
#    Y              x
#    Z              Y
#rotation matrix: tVect=R*tool0_vects
#
#        	R= 0 0 1
#                  1 0 0
#                  0 1 0
    
    
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
    rospy.Service('aruco_service', aruco_service, callback_service)
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
