#!/usr/bin/env python

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
    marker_size = 0.0992 # [meters]
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    static_markers_sked = [10, 11, 12]
    static_poses = (
        (0.31 + marker_size / 2, 0.09 + marker_size / 2, 0),
        (marker_size / 2, marker_size / 2, 0.055),
        (marker_size / 2, 0.285 + marker_size / 2, 0.036)
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

    def prepare_image(self, frame):
        new_frame = frame
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
                for i in range(len(static_markers)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(static_markers[i].corners, self.marker_size, self.camera_mtx, self.camera_dst)
                    static_markers[i] = definedMarker(static_markers[i], rvec, tvec)
                calib_markers = self.global_system(static_markers)
                if self.useGUI:
                    font = cv2.FONT_HERSHEY_PLAIN
                    for i in static_markers:
                        aruco.drawDetectedMarkers(frame, [i.corners])
                    #if len(static_markers):
                    #    aruco.drawAxis(frame, self.camera_mtx, self.camera_dst, np.array(tuple(m.rvec for m in static_markers)), np.array(tuple(m.tvec for m in static_markers)), 0.03)
                    
                    for i, m in enumerate(static_markers):
                        cv2.putText(frame, str(m), (10, 30 * (i + 1)), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    if not calib_markers is None:
                        for i, m in enumerate(calib_markers):
                            cv2.putText(frame, str(m.id_field), m.center, font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    resized = cv2.resize(frame, (1280, 960), interpolation=cv2.INTER_AREA)
                    cv2.imshow("Detected markers", resized)
                    cv2.waitKey(1)

    '''
    m1  m2
    #   #
    
    #
    m3
    '''
    def global_system(self, markers):
        # define cubes on field
        if len(markers) != 3:
            #rospy.logwarn("Non-compliance with the required number of markers")
            return
        sort = sorted(markers, key=lambda m: m.dst, reverse=True)
        m1 = min(sort[:2], key=lambda m: m.tvec[0][0])
        #m1 = sort[0]
        #m2 = sort[1]
        m2 = max(sort[:2], key=lambda m: m.tvec[0][0])
        m3 = sort[2]
        m1.id_field = 1
        m2.id_field = 2
        m3.id_field = 3
        static_alpha = atan2(self.static_poses[2][2] - self.static_poses[1][2], self.static_poses[2][1] - self.static_poses[1][1])
        print(degrees(static_alpha))
        alpha = asin((m2.tvec[0][1] - m3.tvec[0][1]) / (self.static_poses[2][1] -self.static_poses[1][1])) - static_alpha
        cam_y = m2.tvec[0][2] * cos(alpha) + self.static_poses[1][1]
        cam_z = -cam_y * tan(alpha)

        beta = asin((m2.tvec[0][1] - m2.tvec[0][1]) / (self.static_poses[2][1] -self.static_poses[1][1]))
        #print(m1.tvec[0][2] - m2.tvec[0][2])
        print(degrees(alpha), cam_y, cam_z)
        return (m1, m2, m3)

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
