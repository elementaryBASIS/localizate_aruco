#!/usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np
from math import *
import rospy
from time import time
import entities
import numpy as np

class RobotsLocalizer:
    rmarker_size = 0.1 # size of robot marker [meters]
    robots_markers_sked = [14]

    def __init__(self, mtx, dst):
        self.camera_mtx = mtx
        self.camera_dst = dst

    def locate_robots(self, markers, static_rvec, static_tvec):
        robots_markers = list(filter(lambda x: x.id in self.robots_markers_sked, markers))
        
        for i in range(len(robots_markers)):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(robots_markers[i].corners, self.rmarker_size, self.camera_mtx, self.camera_dst)
            robots_markers[i] = entities.DefinedMarker(robots_markers[i], rvec, tvec)
            robots_markers[i].name = "robot_" + str(i)

        robots_pos = []
        for marker in robots_markers:
            tvec = np.array(marker.tvec.T - static_tvec)
            rotM = cv2.Rodrigues(static_rvec)[0]
            r_pos = -np.matrix(rotM).T * np.matrix(tvec)
            r_pos = -r_pos.A1
            robots_pos.append(r_pos)
        return robots_pos, robots_markers