#!/usr/bin/env python

from sys import flags
import cv2 as cv
from cv2 import aruco
import numpy as np
from math import *
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import time
import entities
import stabilizeCam
import localizer
from std_srvs.srv import Empty

class StaticMarkers:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    def __init__(self, calibFilename, staticPosFilename, debug=False, useGUI=False):
        self.debug = debug  # show debug info
        self.useGUI = useGUI
        cv_file = cv.FileStorage(calibFilename, cv.FILE_STORAGE_READ)
        self.camera_mtx = cv_file.getNode("camera_matrix").mat()
        self.camera_dst = cv_file.getNode("dist_coeff").mat()
        assert (not self.camera_mtx is None or not self.camera_dst is None), "%s not found!" % calibFilename
        self.detector_params = aruco.DetectorParameters_create()
        self.staticLocalizer = stabilizeCam.StabilizeCam(self.camera_mtx, self.camera_dst, staticPosFilename)
        self.robotsLocalizer = localizer.RobotsLocalizer(self.camera_mtx, self.camera_dst)

    def main(self):
        filtered_markers = []
        robots = []
        markers = self.detect_markers()
        if not self.staticLocalizer.isCalibrated:
            if not self.staticLocalizer.calibrating:
                rospy.logwarn_throttle_identical(2, "Camera position not defined, calibrate camera")
            is_done, static_markers = self.staticLocalizer.processCalibrate(markers)
            if not is_done:
                self.draw_info(static_markers)
                return
        ret, camera_pos, static_rvec, static_tvec = self.staticLocalizer.camera_position()
        robots, robots_markers = self.robotsLocalizer.locate_robots(markers, static_rvec, static_tvec)
        filtered_markers += robots_markers
        self.draw_info(filtered_markers, robots)
        

    def detect_markers(self):
        frame = self.frame
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        detected_markers = []
        corners, ids, _rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        if ids is None:
            rospy.logwarn_throttle_identical(2, "Don't see any markers")
        else:
            for i in range(len(ids)):
                detected_markers.append(entities.Marker(ids[i], corners[i]))
        return detected_markers

    def recalibrate(self, _):
        self.staticLocalizer.calibrate()
        return []

    def prepare_image(self, frame):
        new_frame = frame
        return new_frame

    def draw_info(self, markers, robots = []):
        if self.useGUI:
            frame = self.frame.copy()
            font = cv.FONT_HERSHEY_PLAIN
            for i, m in enumerate(markers):
                aruco.drawDetectedMarkers(frame, [m.corners])
                cv.putText(frame, str(m.name), m.center, font, 1, (0, 255, 0), 2, cv.LINE_AA)
            ret, camera_pos, static_rvec, static_tvec = self.staticLocalizer.camera_position()
            if not ret:
                resized = cv.resize(frame, (1280, 960), interpolation=cv.INTER_AREA)
                cv.imshow("Detected markers", resized)
                cv.waitKey(1)
                return
            aruco.drawAxis(frame, self.camera_mtx, self.camera_dst, static_rvec, static_tvec, 0.5)
            self.staticLocalizer.draw_zone(frame)
            cv.putText(frame, "Cam: X={0:3.4f} Y={1:3.4f} Z={2:3.4f}" .format(*camera_pos), (5, 30), font, 1.5, (255, 0, 0), 2, cv.LINE_AA)
            for i, robot in enumerate(robots):
                cv.putText(frame, "Robot: X={0:3.4f} Y={1:3.4f} Z={2:3.4f}" .format(*robot), (5, 400 + 30 * i), font, 1.5, (0, 0, 255), 2, cv.LINE_AA)
            
            resized = cv.resize(frame, (1280, 960), interpolation=cv.INTER_AREA)
            cv.imshow("Detected markers", resized)
            cv.waitKey(1)

    def frame_cb(self, data):
        # skip frame if it's corrupted
        try:
            frame = CvBridge().imgmsg_to_cv2(data, "bgr8")
            if frame is None:
                rospy.logwarn('Frame is "None"')
                return
            else:
                frame = frame.copy()
        except CvBridgeError as e:
            rospy.logwarn("Corrupted frame")
        else:
            self.frame = self.prepare_image(frame)
            self.main()

if __name__ == "__main__":
    rospy.init_node('elements_detector', anonymous=True)
    assert (rospy.has_param('localizer/camera_params')), 'Ros param "localizer/camera_params" not set'
    configFile = rospy.get_param('localizer/camera_params')
    assert (rospy.has_param('localizer/static_params')), 'Ros param "localizer/static_params" not set'
    staticFile = rospy.get_param('localizer/static_params')
    useGUI = rospy.get_param('localizer/GUI_enable', False)
    rospy.loginfo("Using calib config: " + configFile)
    rospy.loginfo("Use GUI: " + str(useGUI))
    localizer = StaticMarkers(configFile, staticFile, debug = False, useGUI = useGUI)
    rospy.Subscriber("/camera/image_raw", Image, localizer.frame_cb)
    rospy.Service('localizer/recalibrate', Empty, localizer.recalibrate)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()
