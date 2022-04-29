import cv2 as cv
import numpy as np

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
    def getMarker(self, id):
        if id in self.markers.keys():
            return self.markers[id]
