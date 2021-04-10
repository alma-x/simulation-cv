#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo


rospy.init_node('camera_info_listener', anonymous=True)
msg=rospy.wait_for_message('/camera_image/color/camera_info',CameraInfo)
print(np.reshape(msg.K,[3,3]))

print(np.matrix([[462.1379497504639, 0.0, 320.5],\
                        [0.0, 462.1379497504639, 240.5],\
                        [0.0000e+00, 0.0000e+00, 1.0000e+00]]))