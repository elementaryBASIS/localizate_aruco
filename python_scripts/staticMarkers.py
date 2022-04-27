#!/usr/bin/env python

from sys import flags
import cv2
from cv2 import aruco
import numpy as np
from math import *
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import time

class marker:
    def __init__(self, id, corners):
        self.id = id
        self.corners = corners.copy()
        M = cv2.moments(corners)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        self.center = (cX, cY)

    def __str__(self):
        return "ID: %x" % (self.id)

class definedMarker(marker):
    def __init__(self, marker, rvec, tvec):
        self.__dict__ = marker.__dict__.copy()
        self.rvec = rvec[0]
        self.tvec = tvec[0]
        self.dst = np.linalg.norm(self.tvec, 2)
        self.id_field = None
    def __str__(self):
        return "ID: %x Pose: %s  Ang: %s(deg)" % (self.id, str(self.tvec[0]), str(np.rad2deg(self.rvec[0])))

class staticMarkers:
    marker_size = 0.1 # [meters]
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    static_markers_sked = [10, 11, 12]
    robots_markers_sked = [14]
    static_poses = (
        (0.812 + marker_size / 2, 0.027 + marker_size / 2, 0.0),
        (0.038 + marker_size / 2, 0.284 + marker_size / 2, 0.0),
        (0.637 + marker_size / 2, 0.405 + marker_size / 2, 0.0)
    )

    def __init__(self, calibFileName="calibration_settings.yaml", debug=False, useGUI=False):
        self.debug = debug  # show debug info
        self.useGUI = useGUI
        cv_file = cv2.FileStorage(calibFileName, cv2.FILE_STORAGE_READ)
        self.camera_mtx = cv_file.getNode("camera_matrix").mat()
        self.camera_dst = cv_file.getNode("dist_coeff").mat()
        assert (
            not self.camera_mtx is None or not self.camera_dst is None), "%s not found!" % calibFileName
        self.detector_params = aruco.DetectorParameters_create()
        self.camera_pos = None

    def prepare_image(self, frame):
        new_frame = frame
        print(frame.shape)
        return new_frame

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
            if self.useGUI:
                resized = cv2.resize(frame, (1200, 960), interpolation=cv2.INTER_AREA)
                cv2.imshow("Original frame", resized)
                cv2.waitKey(1)
            frame = self.prepare_image(frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
            corners, ids, _rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
            if ids is None:
                rospy.logwarn("Don't see any markers")
            else:
                detected_markers = []
                for i in range(len(ids)):
                    detected_markers.append(marker(ids[i], corners[i]))
                static_markers = list(filter(lambda x: x.id in self.static_markers_sked, detected_markers))
                robots_markers = list(filter(lambda x: x.id in self.robots_markers_sked, detected_markers))
                for i in range(len(static_markers)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(static_markers[i].corners, self.marker_size, self.camera_mtx, self.camera_dst)
                    static_markers[i] = definedMarker(static_markers[i], rvec, tvec)
                for i in range(len(robots_markers)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(robots_markers[i].corners, self.marker_size, self.camera_mtx, self.camera_dst)
                    robots_markers[i] = definedMarker(robots_markers[i], rvec, tvec)
                self.camera_pos, self.static_rvec, self.static_tvec = self.camera_position(static_markers)
                #print(np.rad2deg(static_rvec))
                robots = []
                for robot in robots_markers:
                    robot_pos = self.locate_robot(robot)
                    if not robot_pos is None:
                        robots.append((robot, robot_pos))
                print(robots)
                if self.useGUI:
                    font = cv2.FONT_HERSHEY_PLAIN
                    for i in static_markers:
                        aruco.drawDetectedMarkers(frame, [i.corners])
                    for i in robots_markers:
                        aruco.drawDetectedMarkers(frame, [i.corners])
                    #if len(static_markers):
                    #    aruco.drawAxis(frame, self.camera_mtx, self.camera_dst, np.array(tuple(m.rvec for m in static_markers)), np.array(tuple(m.tvec for m in static_markers)), 0.03)
                    if not self.static_rvec is None and not self.static_tvec is None:
                        aruco.drawAxis(frame, self.camera_mtx, self.camera_dst, self.static_rvec, self.static_tvec, 1)
                    for i, m in enumerate(static_markers):
                        cv2.putText(frame, str(m.id_field), m.center, font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame, "Cam: " + str(self.camera_pos), (5, 400), font, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame, "Robot: " + str(robots[0][1]), (5, 430), font, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    resized = cv2.resize(frame, (1280, 960), interpolation=cv2.INTER_AREA)
                    cv2.imshow("Detected markers", resized)
                    cv2.waitKey(1)

    '''
    m1  m2
    #   #
    
    #
    m3
    '''
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

    def locate_robot(self, marker):
        if self.camera_pos is None:
            rospy.logwarn("Camera position not defined, calibrate camera")
            return
        tvec = np.array(marker.tvec.T - self.static_tvec)
        rotM = cv2.Rodrigues(self.static_rvec)[0]
        r1_pos = -np.matrix(rotM).T * np.matrix(tvec)
        r1_pos = -r1_pos.A1
        return r1_pos
    '''
    def locate_robot(self, marker):
        if self.camera_pos is None:
            rospy.logwarn("Camera position not defined, calibrate camera")
            return
        #print(r1.rvec, r1.tvec)
        rvec = np.array([marker.rvec] - self.static_rvec.T)
        tvec = np.array([marker.tvec] - self.static_tvec.T)
        print(np.array([marker.tvec]) )
        print(self.static_tvec.T)
        rotM = -np.matrix(cv2.Rodrigues(rvec)[0]).T
        r1_pos = np.matrix(tvec) * rotM
        r1_pos = r1_pos.A1
        return r1_pos
    '''

if __name__ == "__main__":
    rospy.init_node('elements_detector', anonymous=True)
    configFile = rospy.get_param('localizer/camera_params')
    useGUI = rospy.get_param('localizer/GUI_enable', False)
    rospy.loginfo("Using calib config: " + configFile)
    rospy.loginfo("Use GUI: " + str(useGUI))
    detector = staticMarkers(configFile, debug = False, useGUI = useGUI)
    rospy.Subscriber("/camera/image_raw", Image, detector.frame_cb)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
