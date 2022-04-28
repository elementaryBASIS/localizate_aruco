#!/usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np
from math import *
import rospy
from time import time
import entities
import numpy as np

class stabilizeCam:
    measurements_count = 100
    static_markers_sked = [10, 11, 12] # list of static marker ids
    smarker_size = 0.1 # size of static marker [meters]
    static_poses = (
        (0.812 + smarker_size / 2, 0.027 + smarker_size / 2, 0.0),
        (0.038 + smarker_size / 2, 0.284 + smarker_size / 2, 0.0),
        (0.637 + smarker_size / 2, 0.405 + smarker_size / 2, 0.0)
    )
    def __init__(self, mtx, dst, filename="calib.yaml"):
        self.camera_mtx = mtx
        self.camera_dst = dst
        self.save_file = filename
    
    def camera_position(self, markers):
        static_markers = list(filter(lambda x: x.id in self.static_markers_sked, markers))
        for i in range(len(static_markers)):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(static_markers[i].corners, self.smarker_size, self.camera_mtx, self.camera_dst)
            static_markers[i] = entities.definedMarker(static_markers[i], rvec, tvec)
        # define cubes on field
        if len(static_markers) < 3:
            rospy.logwarn("Non-compliance with the required number of markers")
            return
            
        sort = sorted(static_markers, key=lambda m: m.dst, reverse=True)
        m1 = min(sort[:2], key=lambda m: m.tvec[0][0])
        m2 = max(sort[:2], key=lambda m: m.tvec[0][0])
        m3 = sort[2]
        m1.name = "1"
        m2.name = "2"
        m3.name = "3"
        imgPoints = np.array((m1.center, m2.center, m3.center))
        image_points = imgPoints.astype('float32')

        _, rvec, tvec = cv2.solveP3P(np.array(self.static_poses, dtype="float32"), image_points, self.camera_mtx, self.camera_dst, flags=cv2.SOLVEPNP_AP3P )
        if tvec[0][2] > tvec[1][2]:
            rvec = rvec[0]
            tvec = tvec[0]
        else:
            rvec = rvec[1]
            tvec = tvec[1]
        rotM = cv2.Rodrigues(rvec)[0]
        camPos = -np.matrix(rotM).T * np.matrix(tvec)
        camPos = camPos.A1
        return camPos, rvec, tvec, static_markers