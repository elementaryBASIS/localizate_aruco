#!/usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np
from math import *
import rospy
from time import time

class stabilizeCam:
    measurements_count = 100
    static_markers_sked = [10, 11, 12]

    def __init__(self, mtx, dst, filename="calib.yaml"):
        self.camera_mtx = mtx
        self.camera_dst = dst
        self.save_file = filename
    
    def camera_position(self, markers):
        # define cubes on field
        if len(markers) < 3:
            rospy.logwarn("Non-compliance with the required number of markers")
            return None, None, None
        sort = sorted(markers, key=lambda m: m.dst, reverse=True)
        m1 = min(sort[:2], key=lambda m: m.tvec[0][0])
        m2 = max(sort[:2], key=lambda m: m.tvec[0][0])
        m3 = sort[2]
        m1.id_field = 1
        m2.id_field = 2
        m3.id_field = 3
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
        return camPos, rvec, tvec