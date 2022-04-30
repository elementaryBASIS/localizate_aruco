#!/usr/bin/env python

import cv2 as cv
from cv2 import aruco
import numpy as np
from math import *
import rospy
from time import time
import entities
import numpy as np
import configuration

class RobotsLocalizer:
    

    def __init__(self, mtx, dst):
        self.camera_mtx = mtx
        self.camera_dst = dst

    def locate_robots(self, markers, static_rvec, static_tvec):
        for r in configuration.robots:
            r.real_reset()
        for m in markers:
            self.estimatePoseRobot(m)
        for robot in configuration.robots:
            if len(robot.definedMarkers):
                rvec, tvec = robot.get_mean_position()
                r_ang, r_pos = self.relativePosition(rvec, tvec, static_rvec, static_tvec)
                robot.pos  = r_pos.ravel()
                robot.ang = r_ang.ravel()
        return configuration.robots

    def estimatePoseRobot(self, marker):
        marker_pos = None
        for r in configuration.robots:
            ret = r.get_marker(marker.id[0])
            if not ret is None:
                robot = r
                marker_pos = ret
                break
        else:
            return
        _, rvec, tvec = cv.solvePnP(marker_pos, np.array(marker.corners, dtype="float32"), self.camera_mtx, self.camera_dst)
        marker = entities.DefinedMarker(marker, [rvec], [tvec])
        robot.add_DefinedMarker(marker)

    
    def inversePerspective(self, rvec, tvec):
        """ Applies perspective transform for given rvec and tvec. """
        # https://stackoverflow.com/questions/52119190/relative-rotation-between-pose-rvec
        R, _ = cv.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(R, np.matrix(-tvec))
        invRvec, _ = cv.Rodrigues(R)
        return invRvec, invTvec

    def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
        """ Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 """
        # https://stackoverflow.com/questions/52119190/relative-rotation-between-pose-rvec
        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

        # Inverse the second marker, the right one in the image
        invRvec, invTvec = self.inversePerspective(rvec2, tvec2)

        info = cv.composeRT(rvec1, tvec1, invRvec, invTvec)
        composedRvec, composedTvec = info[0], info[1]

        composedRvec = composedRvec.reshape((3, 1))
        composedTvec = composedTvec.reshape((3, 1))
        return composedRvec, composedTvec
