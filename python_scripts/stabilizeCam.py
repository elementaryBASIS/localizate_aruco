#!/usr/bin/env python

import cv2 as cv
from cv2 import aruco
import numpy as np
from math import *
import rospy
from time import time
import entities
import numpy as np

class StabilizeCam:
    measurements_count = 1 # count of frames for calibration
    static_markers_sked = [10, 11, 12] # list of static marker ids
    smarker_size = 0.1 # size of static marker [meters]
    static_poses = (
        (0.812 + smarker_size / 2, 0.027 + smarker_size / 2, 0.0),
        (0.038 + smarker_size / 2, 0.284 + smarker_size / 2, 0.0),
        (0.637 + smarker_size / 2, 0.405 + smarker_size / 2, 0.0)
    )
    field_size = (0.8, 0.6) #size of battlefield [meters]
    def __init__(self, mtx, dst, filename):
        self.camera_mtx = mtx
        self.camera_dst = dst
        self.save_file = filename
        self.isCalibrated = False  
        self.calibrating = False
        self.filename = filename
        self._load_calibration()

    def calibrate(self):
        self.isCalibrated = False
        self.calibrating = True
        self.measurements = {
            "m1" : [],
            "m2" : [],
            "m3" : []
        }

    def processCalibrate(self, markers):
        
        static_markers = list(filter(lambda x: x.id in self.static_markers_sked, markers))
        for i in range(len(static_markers)):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(static_markers[i].corners, self.smarker_size, self.camera_mtx, self.camera_dst)
            static_markers[i] = entities.DefinedMarker(static_markers[i], rvec, tvec)
        # define cubes on field
        if len(static_markers) < 3:
            rospy.logwarn("Non-compliance with the required number of static markers")
            return self.isCalibrated, []
            
        sort = sorted(static_markers, key=lambda m: m.dst, reverse=True)
        m1 = min(sort[:2], key=lambda m: m.tvec[0][0])
        m2 = max(sort[:2], key=lambda m: m.tvec[0][0])
        m3 = sort[2]
        m1.name = "Static_1"
        m2.name = "Static_2"
        m3.name = "Static_3"
        if not self.calibrating:
            return self.isCalibrated, sort

        self.measurements["m1"].append(m1.center)
        self.measurements["m2"].append(m2.center)
        self.measurements["m3"].append(m3.center)
        if len(self.measurements["m1"]) < self.measurements_count:
            return self.isCalibrated, sort
        m1_poses = np.array(self.measurements["m1"])
        m1_pos = (np.mean(m1_poses[:, 0]), np.mean(m1_poses[:, 1]))
        m2_poses = np.array(self.measurements["m2"])
        m2_pos = (np.mean(m2_poses[:, 0]), np.mean(m2_poses[:, 1]))
        m3_poses = np.array(self.measurements["m3"])
        m3_pos = (np.mean(m3_poses[:, 0]), np.mean(m3_poses[:, 1]))

        std = []
        for m in ("m1", "m2", "m3"):
            mment = np.array(self.measurements[m])
            std.append((np.std(mment[:, 0]), np.std(mment[:, 1])))
        std = np.array(std)
        image_points = np.array((m1_pos, m2_pos, m3_pos))
        rospy.loginfo("Mean:\n" + str(image_points))
        rospy.loginfo("Std:\n" + str(std))

        _, rvec, tvec = cv.solveP3P(np.array(self.static_poses, dtype="float32"), image_points, self.camera_mtx, self.camera_dst, flags=cv.SOLVEPNP_AP3P )
        if tvec[0][2] > tvec[1][2]:
            rvec = rvec[0]
            tvec = tvec[0]
        else:
            rvec = rvec[1]
            tvec = tvec[1]
        rotM = cv.Rodrigues(rvec)[0]
        camPos = -np.matrix(rotM).T * np.matrix(tvec)
        camPos = camPos.A1
        self.camPos = camPos
        self.static_rvec = rvec
        self.static_tvec = tvec
        self._zone2img()
        self._save_calibration(image_points, std)
        self.isCalibrated = True
        self.calibrating = False
        return self.isCalibrated, sort
    
    def _zone2img(self):
        x, y = self.field_size
        points_real = np.zeros((4 * 6, 3),  dtype="float32")
        for i in range(6):
            points_real[i] = (0, 0, i / 20.0)
            points_real[6 + i] = (x, 0, i / 20.0)
            points_real[12 + i] = (0, y, i / 20.0)
            points_real[18 + i] = (x, y, i / 20.0)
        self.zone_pixels, _ = cv.projectPoints(points_real, self.static_rvec, self.static_tvec, self.camera_mtx, self.camera_dst)

    def draw_zone(self, img):
        points = self.zone_pixels
        for i in range(6):
            img = cv.line(img, tuple(points[i].ravel()), tuple(points[6 + i].ravel()), (100, 255, 0), 1)
            img = cv.line(img, tuple(points[i].ravel()), tuple(points[12 + i].ravel()), (150, 150, 200), 1)
            img = cv.line(img, tuple(points[18 + i].ravel()), tuple(points[6 + i].ravel()), (200, 150, 200), 1)
            img = cv.line(img, tuple(points[18 + i].ravel()), tuple(points[12 + i].ravel()), (255, 100, 255), 1)
        for i in range(4):
            img = cv.line(img, tuple(points[i * 6].ravel()), tuple(points[i * 6 + 5].ravel()), (255, 0, 0), 2)
        img = cv.line(img, tuple(points[0].ravel()), tuple(points[18].ravel()), (255, 0, 255), 2)
        img = cv.line(img, tuple(points[6].ravel()), tuple(points[12].ravel()), (255, 0, 255), 2)

    def camera_position(self):
        if self.isCalibrated:
            return True, self.camPos, self.static_rvec, self.static_tvec
        else:
            return False, None, None, None

    def _save_calibration(self, mean, std):
        cv_file = cv.FileStorage(self.filename, cv.FILE_STORAGE_WRITE)
        cv_file.write("camera_pos", self.camPos)
        cv_file.write("static_rvec", self.static_rvec)
        cv_file.write("static_tvec", self.static_tvec)
        cv_file.write("mean", mean)
        cv_file.write("std", std)
        cv_file.release()
        rospy.loginfo("Calibration saved to " + self.filename)

    def _load_calibration(self):
        cv_file = cv.FileStorage(self.filename, cv.FILE_STORAGE_READ)
        static_rvec = cv_file.getNode("static_rvec").mat()
        static_tvec = cv_file.getNode("static_tvec").mat()
        camPos = cv_file.getNode("camera_pos").mat()
        if not static_rvec is None and not static_tvec is None and not camPos is None:
            self.static_rvec = static_rvec
            self.static_tvec = static_tvec
            self.camPos = camPos.ravel()
            self._zone2img()
            self.isCalibrated = True
            self.calibrating = False
            rospy.loginfo("Static calibration loaded from " + self.filename)
        else:
            rospy.logwarn("Static calibration file " + self.filename + " doesn't exist or corrupted")