import cv2 as cv
import numpy as np
import rospy
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Vector3

class Marker:
    def __init__(self, id, corners, name = ""):
        self.id = id
        self.name = name
        self.corners = corners.copy()
        M = cv.moments(corners)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        self.center = (cX, cY)

    def __str__(self):
        return "ID: %x" % (self.id)

class DefinedMarker(Marker):
    def __init__(self, marker, rvec, tvec):
        self.__dict__ = marker.__dict__.copy()
        self.rvec = rvec[0]
        self.tvec = tvec[0]
        self.dst = np.linalg.norm(self.tvec, 2)
    def __str__(self):
        return "ID: %x Pose: %s  Ang: %s(deg)" % (self.id, str(self.tvec[0]), str(np.rad2deg(self.rvec[0])))

class Robot:
    '''
        Every robot has prismatical head.
        Note: it isn's cube, that's why we set every corner's position from bottom center for each marker [meters]
        Corners order:
                        0 1
                        3 2
            Counter clockwise from left top
    '''
    markers = dict()

    def __init__(self, name):
        self.name = name
        self.real_reset()
        self.topic = rospy.Publisher('robot/' + self.name, Twist)

    def get_marker(self, id):
        if id in self.markers.keys():
            return self.markers[id]

    def add_DefinedMarker(self, marker):
        self.definedMarkers.append(marker)
    
    def get_mean_position(self):
        if not len(self.definedMarkers):
            return
        tvecs = []
        rvecs = []
        for m in self.definedMarkers:
            tvecs.append(m.tvec)
            rvecs.append(m.rvec)
        tvecs = np.array(tvecs).reshape((len(tvecs), 3))
        rvecs = np.array(rvecs).reshape((len(rvecs), 3))
        tvec = np.mean(tvecs, axis = 0)
        rvec = np.mean(rvecs, axis = 0)
        return rvec, tvec
        
    def real_reset(self):
        self.pos = None
        self.ang = None
        self.definedMarkers = []

    def publish(self):
        if not self.pos is None:
            self.topic.publish(Twist(Vector3(*self.pos), Vector3(*self.ang)))
    
    def __str__(self):
        if self.pos is None or self.ang is None:
            return "{0}: out of sight" .format(self.name)
        return "{0}: X={1:3.4f} Y={2:3.4f} Z={3:3.4f}" .format(self.name, *self.pos)
    
