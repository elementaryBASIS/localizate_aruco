#!/usr/bin/env python

import cv2 as cv
from cv2 import aruco
import numpy as np
from math import *
import rospy
from time import time
import entities
import numpy as np

class RobotsLocalizer:
    rmarker_size = 0.1 # size of robot marker [meters]
    robots_markers_sked = [25]
    robot = [entities.Robot("Doshirak")]
    robot[0].markers[25] = np.array((
            (0.056, 0.102, 0.153),
            (-0.056, 0.102, 0.153),
            (-0.056, 0.102, 0.064),
            (0.056, 0.102, 0.064)
        ), dtype="float32")
        
    def __init__(self, mtx, dst):
        self.camera_mtx = mtx
        self.camera_dst = dst

    def locate_robots(self, markers, camera_pos, static_rvec, static_tvec):
        robots_markers = list(filter(lambda x: x.id in self.robots_markers_sked, markers))
        for i in range(len(robots_markers)):
            rvec, tvec = self.estimatePose(robots_markers[i])
            robots_markers[i] = entities.DefinedMarker(robots_markers[i], rvec, tvec)
            robots_markers[i].name = "robot_" + str(i)
        robots_pos = []
        for marker in robots_markers:
            rotM = cv.Rodrigues(static_rvec)[0]
            r_pos = -np.matrix(rotM).T * np.matrix(np.array(marker.tvec))
            r_pos = camera_pos - r_pos.A1 
            robots_pos.append(r_pos)
        return robots_pos, robots_markers

    def estimatePose(self, marker):
        marker_pos = None
        for r in self.robot:
            ret = r.getMarker(marker.id[0])
            if not ret is None:
                marker_pos = ret
                break
        else:
            return
        _, rvec, tvec = cv.solvePnP(marker_pos, np.array(marker.corners, dtype="float32"), self.camera_mtx, self.camera_dst)
        return [rvec], [tvec]