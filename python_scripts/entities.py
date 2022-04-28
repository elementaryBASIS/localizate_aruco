import cv2
import numpy as np

class marker:
    def __init__(self, id, corners, name = ""):
        self.id = id
        self.name = name
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
    def __str__(self):
        return "ID: %x Pose: %s  Ang: %s(deg)" % (self.id, str(self.tvec[0]), str(np.rad2deg(self.rvec[0])))

class robot:
    def __init__(self):
        pass