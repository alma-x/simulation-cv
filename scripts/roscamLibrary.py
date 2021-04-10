#!/usr/bin/env python
import cv2 as cv
#from cv2 import aruco as aruco
import cv2.aruco as aruco
#import rospy
#from std_msgs.msg import String
#from PIL import Image
#import matplotlib.pyplot as plt
import numpy as np
#from sensor_msgs.msg import Image as sensImg
#from sensor_msgs.msg import CompressedImage as CsensImg
#frofm sensor_msgs.msg import PointCloud2 as sensPCld
#from std_msgs.msg import Float64MultiArray



#################### COSMETHIC FUNCTIONS ############################




########### GEOMETRIC FUNCTIONS ####################

    
    
##############################################################################
        

def IdOverAruco(ids, corners, grayQueryImg):
    font = cv.FONT_HERSHEY_SIMPLEX

    arucoIDImg = np.copy(grayQueryImg)
    # Check if some aruco has been found
    if ids is not None and len(ids) >= 1:
        for i, corner in zip(ids, corners):
            #print("Corners:", corner)
            cv.fillPoly(arucoIDImg, corner.astype(int), (230, 230, 230))

            (textX, textY )= np.abs(corner[0][0] + corner[0][2]) / 2
            textsizeX, textsizeY = cv.getTextSize(str(i[0]), font, 1, 3)[0]
            textX = (textX - textsizeX / 2).astype(int)
            textY = (textY + textsizeY / 2).astype(int)
            cv.putText(arucoIDImg, str(i[0]), (textX, textY), font, 1, (0, 0, 0), 2)

    return arucoIDImg

def distOverAruco(distanceMarker, corners, queryImg):
    font = cv.FONT_HERSHEY_SIMPLEX

    cv.fillPoly(queryImg, corners.astype(int), (230, 230, 230))    
    (textX, textY) = np.abs(corners[0][0] + corners[0][2]) / 2    
    textsizeX, textsizeY = cv.getTextSize(str(distanceMarker), font, 1, 3)[0]    
    textX = (textX - textsizeX / 2).astype(int)
    textY = (textY + textsizeY / 2).astype(int)    
    cv.putText(queryImg, str(distanceMarker), (textX, textY), font, 1, (0, 0, 0), 2)
    return queryImg


#def findRectangles(imgRect, idsR, cornersR):
#    debug = 0
#    # Grayscale image is requested for contour recognition
#    imgRectGray = cv.cvtColor(imgRect, cv.COLOR_BGR2GRAY)
#
#    # Check if at least one marker has been found
#    if idsR is None or len(idsR) == 0:
#        # If no marker detected, exit
#        print("No marker detected!")
#        return None
#
#    # Print found arucos
#    if debug:
#        for i, corner in zip(idsR, cornersR):
#            print('Detected aruco with ID: {}.'.format(i[0]))
#
#    #======== Find contours in image ========
#        
#    # The "findContours" function nedd a binary image, so need to threeshold before
#    #ret, imgThresh = cv2.threshold(imgRectGray, 127, 255, 0)
#    imgThresh = cv.adaptiveThreshold(imgRectGray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 51, 2)
#    contours, hierarchy = cv.findContours(imgThresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
#
#    # Identify rectangular contours
#    rect_cnts = []
#    areas = []
#    for cnt in contours:
#        peri = cv.arcLength(cnt, True)
#        approx = cv.approxPolyDP(cnt, 0.04 * peri, True)
#        #(x, y, w, h) = cv2.boundingRect(cnt)
#        #ar = w / float(h)
#        if len(approx) == 4: # shape filtering condition
#            # Get the area of the rectangle, need to exclude rectangles with area less than the 
#            # one of the smallest aruco
#            area = cv.contourArea(cnt)
#
#            # Exclude rectangles with pixel area, due to some threesholding error perhaps
#            if area >= 5.0:
#                areas.append(area)
#                rect_cnts.append(cnt) # Shape is rectangle, add to the valid list
#    # Now in rect_cnts[] we have only rectangular contours
#
#    #======== Discard the contours that do not contain any aruco (multiple markers can be present in the image)
#
#    # Make a copy to preserve the original image, draw functions are destructive
#    imgRectDraw = np.copy(imgRect)
#
#    j = 0
#    in_cnt = [] # dim (2,2) array of markers containing array of contours for each marker
#    for aruco_n, corner_n in zip(idsR, cornersR): # for every aruco marker in image...
#        cnt_father = [] # collect contours, for each marker
#        corner_n = corner_n[0].astype(int) # adjust array dimensionality
#        if debug:
#            imgRectDraw = cv.circle(imgRectDraw, (corner_n[0][0], corner_n[0][1]), 50, (0,0,255), 3)
#        i=0
#        for cnt in rect_cnts: # for every rectangular contour...
#            dist = cv.pointPolygonTest(cnt, (corner_n[0][0], corner_n[0][1]), True) # Check if top left corner of the aruco
#            # dist is:
#            # - dist<0 if point is outside contour
#            # - dist=0 if point is in the contour itself
#            # - dist>0 if point is inside
#            # Note that the ==0 is not exactly zero, can be 0.5, so a threshold is needed
#            # Check difference in area: must be 10% greater than the one of the aruco
#            if (dist > 1.) and (areas[i] > cv.contourArea(corner_n)*1.20): # if the aruco is inside the contour...
#                cnt_father.append(cnt) # add the contour in list
#                if debug:
#                    print("Contour distance:", dist)
#                    cv.drawContours(imgRectDraw, [cnt], -1, (0,255,0), 2) # for debug draw the contour found
#            i+=1
#        if len(cnt_father) != 0:
#            in_cnt.append(cnt_father) # check next aruco
#        
#    return in_cnt


#def computeDistance(imgShape,corners,marker_real_world_mm, debug=0):
#    # ====== Camera parameters ==========
#    # Sensor is 5.64mm wide
#    # Original resolution is 4032x1960
#    focal_lenght = 3558.572811
#    # ====== End camera parameters ======
#
#    # Size of the square marker
##     marker_real_world_mm = 46
#
##     imgShape = queryImg.shape
#    
#    marker_dim_px = np.sqrt((corners[0][0][0][0] - corners[0][0][3][0])**2 + (corners[0][0][0][1] - corners[0][0][3][1])**2)
#        
#    distance_mm = marker_real_world_mm * (np.max(imgShape) / 4032) * focal_lenght / marker_dim_px
#    if debug: print("Distance: {}cm".format(round(distance_mm / 10, 1)))
#    return distance_mm


#def draw_axis_on_marker(queryImg,corners, ids):
#    
#    # Create a square on flat plane with dimension of 4,6 cm
#    marker_square = np.float32([[0, 0, 0], [0, 4.6, 0], [4.6, 4.6, 0], [4.6, 0, 0]])
#    # Array for drawing the 3 cartesian axes
#    axis = np.float32([[3,0,0], [0,3,0], [0,0,3]]).reshape(-1,3)
#    # assuming one marker
#    the_marker = corners#[0]
#    # Find the rotation and translation vectors
#    ret, rvecs, tvecs = cv.solvePnP(marker_square, the_marker[0], camera_matrix, camera_dist_coefs)
#
#    # Project axes points according to camera matrix and distortion coeff
#    imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, camera_matrix, camera_dist_coefs)
#    # Draw axes in the corner of the marker
#    for marker in the_marker:
#        drawnImg = draw(queryImg, marker, imgpts)
#
#    # From rvecs compute the rotation matrix
#    rotation_matrix = cv.Rodrigues(rvecs)[0]
#    # Get the Projection matrix
#    P = np.hstack((rotation_matrix, tvecs))

    # Compute eulero angles in degree
#    euler_angles_degrees = - cv.decomposeProjectionMatrix(P)[6]
#    euler_angles_radians = euler_angles_degrees * np.pi / 180
#     print(euler_angles_degrees)
#
#    return drawnImg

def computeDistanceSingle(imgShape,corners,marker_real_world_mm, debug=0):
    # ====== Camera parameters ==========
    # Sensor is 5.64mm wide
    # Original resolution is 4032x1960
    focal_lenght = 426
    # ====== End camera parameters ======

    # Size of the square marker
#     marker_real_world_mm = 46

#     imgShape = queryImg.shape
    
    marker_dim_px = np.sqrt((corners[0][0][0] - corners[0][3][0])**2 + (corners[0][0][1] - corners[0][3][1])**2)
    
        
    distance_mm = marker_real_world_mm * (np.max(imgShape) / 640) * focal_lenght / marker_dim_px
    if debug: print("Distance: {}cm".format(round(distance_mm / 10, 1)))
    return distance_mm

def drawSingleAru(queryImg, corners, imgpts):
    corner = tuple(corners.ravel())
    queryImg = cv.line(queryImg, corner, tuple(imgpts[0].ravel()), (255,0,0), 3)
    queryImg = cv.line(queryImg, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
    queryImg = cv.line(queryImg, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
    return queryImg
    



# def cut_markers_area(queryImg,corners,rotation_matrix):
    
#         rotatedImg = cv2.warpAffine(queryImg, rotation_matrix, _img_rot.shape[1::-1], flags=cv2.INTER_LINEAR)
#         _extrema = cv2.perspectiveTransform(np.array([maxx,maxy,minx,miny]), _rot_mat)
                
#         img_mrk.append(_img_rot[_extrema[0][1]:_extrema[1][1], _extrema[0][0]:_extrema[1][0], :])
        
#         i+=1
#     return img_mrk


def singleAruRelPos(queryImg,corners,Id,markerSize_mm,camera_matrix, camera_dist_coefs,
                    focal_length,superimpAru='none'):
   
    imgShape = queryImg.shape
    
    markerDim_px = np.sqrt((corners[0][0][0] - corners[0][3][0])**2 + (corners[0][0][1] - corners[0][3][1])**2)
    distnc_mm = markerSize_mm * (np.max(imgShape) / 640) * focal_length / markerDim_px
    
    mrkSiz_cm= round(markerSize_mm/10,1)
    markerSquare_cm = np.float32([[0, 0, 0], [0, mrkSiz_cm, 0], [mrkSiz_cm, mrkSiz_cm, 0], [mrkSiz_cm, 0, 0]])
    _, rvecs, tvecs = cv.solvePnP(markerSquare_cm, corners, camera_matrix, camera_dist_coefs)
#     rvecs,tvecs,_= aruco.estimatePoseSingleMarkers(corners,markerSize_mm,camera_matrix,camera_dist_coefs)
#     r & tvects are different from the ones with previous code
    
    rotation_matrix = cv.Rodrigues(rvecs)[0]# From rvecs compute the rotation matrix
    P = np.hstack((rotation_matrix, 10*tvecs))# Get the Projection matrix
    
    # Project axes points according to camera matrix and distortion coeff
    axis = np.float32([[3,0,0], [0,3,0], [0,0,3]]).reshape(-1,3)# Array for drawing the 3 cartesian axes
    imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, camera_matrix, camera_dist_coefs)
    
#     cv.drawContours(queryImg, [corners], 0, (0,255,0), 3)#gotta work on this
    if superimpAru=='distance': queryImg=distOverAruco(round(distnc_mm, 1),corners,queryImg)
    elif superimpAru=='marker': queryImg=IdOverAruco(Id,corners,queryImg)
        
    queryImg = drawSingleAru(queryImg, corners[0][0], imgpts)#this solution works better than the following
#     queryImg = aruco.drawAxis(queryImg, camera_matrix, camera_dist_coefs, rvecs, tvecs, 2)
  


    centerx,centery=center=np.abs(corners[0][0] + corners[0][2])/2
    #
#    (mrkSiz_cm/2,mrkSiz_cm/2)
#    _, rvecsCent, tvecsCent = cv.solvePnP([[[mrkSiz_cm/2, mrkSiz_cm/2]]], [[[centerx, centery]]], camera_matrix, camera_dist_coefs)
#    print('center t vecs',tvecsCent)
#    euler_angles_degrees = - cv.decomposeProjectionMatrix(P)[6]
#    euler_angles_radians = euler_angles_degrees * np.pi / 180
    
    queryImg=cv.circle(queryImg, (int(centerx),int(centery)),5,(255,255,0),-1)    
    return queryImg,distnc_mm,P,rvecs


def nsingleAruRelPos(queryImg,corners,Id,markerSize_mm,camera_matrix, camera_dist_coefs,
                    focal_length,superimpAru='none'):
   
    
    rvecs,tvecs,_= aruco.estimatePoseSingleMarkers(corners,markerSize_mm,camera_matrix,camera_dist_coefs)
#     r & tvects are different from the ones with previous code
    distnc_mm=np.sqrt((tvecs[0])**2+(tvecs[1])**2+(tvecs[2])**2)
    
    rotation_matrix = cv.Rodrigues(rvecs)[0]# From rvecs compute the rotation matrix
    P = np.hstack((rotation_matrix, 10*tvecs))# Get the Projection matrix
    
    # Project axes points according to camera matrix and distortion coeff
    axis = np.float32([[3,0,0], [0,3,0], [0,0,3]]).reshape(-1,3)# Array for drawing the 3 cartesian axes
    imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, camera_matrix, camera_dist_coefs)
    
#     cv.drawContours(queryImg, [corners], 0, (0,255,0), 3)#gotta work on this
    if superimpAru=='distance': queryImg=distOverAruco(round(distnc_mm, 1),corners,queryImg)
    elif superimpAru=='marker': queryImg=IdOverAruco(Id,corners,queryImg)
        
#    queryImg = drawSingleAru(queryImg, corners[0][0], imgpts)#this solution works better than the following
    aruco.drawAxis(queryImg, camera_matrix, camera_dist_coefs, rvecs, tvecs, 2)
     
    
#    for ii in(range(length(rvecs)))
#       aruco.drawAxis(queryImg, camera_matrix, camera_dist_coefs, rvecs[ii-1], tvecs[ii-1], 0.1);

#    centerx,centery=center=np.abs(corners[0][0] + corners[0][2])/2
    #
#    (mrkSiz_cm/2,mrkSiz_cm/2)
#    _, rvecsCent, tvecsCent = cv.solvePnP([[[mrkSiz_cm/2, mrkSiz_cm/2]]], [[[centerx, centery]]], camera_matrix, camera_dist_coefs)
#    print('center t vecs',tvecsCent)
#    euler_angles_degrees = - cv.decomposeProjectionMatrix(P)[6]
#    euler_angles_radians = euler_angles_degrees * np.pi / 180
    
#    queryImg=cv.circle(queryImg, (int(centerx),int(centery)),5,(255,255,0),-1)    
    return queryImg,distnc_mm,P,rvecs