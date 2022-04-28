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
import entities
import stabilizeCam

class staticMarkers:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    robots_markers_sked = [14]    
    rmarker_size = 0.1 # size of robot marker [meters]
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

    def main(self):
        filtered_markers = []
        robots = []
        markers = self.detect_markers()
        static = stabilizeCam.stabilizeCam(self.camera_mtx, self.camera_dst)
        cp_ret = static.camera_position(markers)
        if cp_ret is None:
            self.draw_info(markers, robots)
            return
        self.camera_pos, self.static_rvec, self.static_tvec, static_markers = tuple(cp_ret)
        filtered_markers += static_markers
        robots, robots_markers = self.locate_robots(markers)
        filtered_markers += robots_markers
        self.draw_info(filtered_markers, robots)

    def detect_markers(self):
        frame = self.frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected_markers = []
        corners, ids, _rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        if ids is None:
            rospy.logwarn("Don't see any markers")
        else:
            for i in range(len(ids)):
                detected_markers.append(entities.marker(ids[i], corners[i]))
        return detected_markers

    def prepare_image(self, frame):
        new_frame = frame
        return new_frame

    def draw_info(self, markers, robots):
        if self.useGUI:
            frame = self.frame.copy()
            font = cv2.FONT_HERSHEY_PLAIN
            
            for i in markers:
                aruco.drawDetectedMarkers(frame, [i.corners])
            if self.camera_pos is None:
                resized = cv2.resize(frame, (1280, 960), interpolation=cv2.INTER_AREA)
                cv2.imshow("Detected markers", resized)
                cv2.waitKey(1)
                return
            aruco.drawAxis(frame, self.camera_mtx, self.camera_dst, self.static_rvec, self.static_tvec, 1)
            for i, m in enumerate(markers):
                cv2.putText(frame, str(m.name), m.center, font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, "Cam: X={0:3.4f} Y={1:3.4f} Z={2:3.4f}" .format(*self.camera_pos), (5, 30), font, 1.5, (255, 0, 0), 2, cv2.LINE_AA)
            for i, robot in enumerate(robots):
                cv2.putText(frame, "Robot: X={0:3.4f} Y={1:3.4f} Z={2:3.4f}" .format(*robot), (5, 400 + 30 * i), font, 1.5, (0, 0, 255), 2, cv2.LINE_AA)
            
            resized = cv2.resize(frame, (1280, 960), interpolation=cv2.INTER_AREA)
            cv2.imshow("Detected markers", resized)
            cv2.waitKey(1)
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
            '''
            if self.useGUI:
                
            '''


    def locate_robots(self, markers):
        if self.camera_pos is None:
            rospy.logwarn("Camera position not defined, calibrate camera")
            return []
        robots_markers = list(filter(lambda x: x.id in self.robots_markers_sked, markers))
        
        for i in range(len(robots_markers)):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(robots_markers[i].corners, self.rmarker_size, self.camera_mtx, self.camera_dst)
            robots_markers[i] = entities.definedMarker(robots_markers[i], rvec, tvec)
            robots_markers[i].name = "robot_" + str(i)

        robots_pos = []
        for marker in robots_markers:
            tvec = np.array(marker.tvec.T - self.static_tvec)
            rotM = cv2.Rodrigues(self.static_rvec)[0]
            r_pos = -np.matrix(rotM).T * np.matrix(tvec)
            r_pos = -r_pos.A1
            robots_pos.append(r_pos)
        return robots_pos, robots_markers
        
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
