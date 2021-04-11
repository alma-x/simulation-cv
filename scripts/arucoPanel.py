#!/usr/bin/env python

import rospy
import numpy as np

#from std_msgs.msg import String
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CameraInfo
from ur3_control.srv import aruco_service,aruco_serviceResponse

import cv2 as cv
import cv2.aruco as aruco
from cv_bridge import CvBridge
from roscamLibrary_new import singleAruRelPos

#PUBLISHER DI ARRAY:
#aruco_position_pub = rospy.Publisher('/almax/aruco_target',Float64MultiArray,queue_size=20)
#array = [69.1,0,1,33,1,1,1,0]
#robaccia = Float64MultiArray(data=array)
#aruco_position_pub.publish(robaccia)
#------------------------------------------------

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    
#----------------------------------------------
    
def loadCameraParam(myCam):
    global cameraMatr
    global cameraDistCoefs
    global cameraFocLen
    
    print('loading camera parameters...')
    cameraInfoMsg=rospy.wait_for_message(myCam+'/color/camera_info',CameraInfo)
    cameraMatr=np.reshape(cameraInfoMsg.K,[3,3])
    cameraDistCoefs=cameraInfoMsg.D
    cameraFocLen=np.mean([np.ravel(cameraMatr[0])[0],np.ravel(cameraMatr[1])[1]])
    
#------------------------------------------------------------
    
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

#def loadTargetRequest():
#    selectedTarget=read_somewhere(targetRequestTopic)
#    (targetMarkId,targetMarkSize)=targetMarker=aruTargetDict[selectedTarget]

#----------------------------------------
bridge=CvBridge()
 
def callbackRaw(raw_img):
    global aruco_success
    global msgVector
    global msgRotMatrix
      
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv.cvtColor(cv_image,cv.COLOR_RGB2GRAY)
    
    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    if detIds is not None and len(detIds) >= 1:
        
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
            
        for mId, aruPoints in zip(detIds, detCorners):
                
            detAruImg,aruDistnc,Pmatr=singleAruRelPos(detAruImg,aruPoints,mId,targetMarkSize,
                                          cameraMatr,cameraDistCoefs,superimpAru='distance',
                                          tglDrawCenter=0,tglDrawMark=1)
            rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
            
#            comparison between currently found marker and target
            if mId==targetMarkId:
                aruco_success=True
                msgRotMatrix=rotMatr
                msgVector=tVect
            else:
                aruco_success=False
    else:
        aruco_success=False
        detAruImg=cv_image.copy()

    cv.imshow('detected markers',detAruImg)
    
    key = cv.waitKey(12) & 0xFF# key still unused
#    if key == 27:# 27:esc, ord('q'):q
#       exit_somehow()

    
#-----------------------------------------------------------------

aruco_success=False
msgVector=[0,0,0]#np.zeros([numRows,numCols])
msgRotMatrix=[[0,0,0],[0,0,0],[0,0,0]]

        
tglWristLengthRecovery=1
# recovered percentage
recovLenRatio=1

def callback_service(req):
    global aruco_success
    global msgVector
    global msgRotMatrix
#    if aruco_success: print("ARUCO SUCCESS:TRUE")
    
    return aruco_serviceResponse(
        success=aruco_success,
        x=0.001*msgVector[2] +(recovLenRatio*0.08 if tglWristLengthRecovery else 0),#[m]
        y=0.001*msgVector[0],   
        z=0.001*msgVector[1],
        vector=np.ravel(msgRotMatrix)#flattened array
        )


#------------------------------------------------------

    
def listener(myCam,myTop,myType,myCallk):    
    rospy.init_node('camera_listener', anonymous=True)
    loadCameraParam(myCam)
    print('ready')
    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
    rospy.Service('aruco_service', aruco_service, callback_service)
    try:
        rospy.spin()
    except KeyboardInterrupt:#
        print('Closing')
    cv.destroyAllWindows()
    
#---------------------------------------------------------------
    
    
camDict={'moving':"/camera_image",
            'fixed':"/camera_image_fix"}

topicDict={'raw':("/color/image_raw",
                    sensImg,
                    callbackRaw)    
            }   

if __name__ == '__main__':
    myCamera=camDict['moving']
    myTopicFull=topicDict['raw']
    
    print('connecting to:'+myCamera+myTopicFull[0]+'...')
    listener(myCamera,myTopicFull[0],myTopicFull[1],myTopicFull[2])

