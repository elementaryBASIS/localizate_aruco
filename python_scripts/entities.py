<<<<<<< HEAD
import cv2 as cv
import numpy as np

class Marker:
=======
import cv2
import numpy as np

class marker:
>>>>>>> 1d80a791a3865431e88396f9ca970d23401cda74
    def __init__(self, id, corners, name = ""):
        self.id = id
        self.name = name
        self.corners = corners.copy()
<<<<<<< HEAD
        M = cv.moments(corners)
=======
        M = cv2.moments(corners)
>>>>>>> 1d80a791a3865431e88396f9ca970d23401cda74
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        self.center = (cX, cY)

    def __str__(self):
        return "ID: %x" % (self.id)

<<<<<<< HEAD
class DefinedMarker(Marker):
=======
class definedMarker(marker):
>>>>>>> 1d80a791a3865431e88396f9ca970d23401cda74
    def __init__(self, marker, rvec, tvec):
        self.__dict__ = marker.__dict__.copy()
        self.rvec = rvec[0]
        self.tvec = tvec[0]
        self.dst = np.linalg.norm(self.tvec, 2)
    def __str__(self):
        return "ID: %x Pose: %s  Ang: %s(deg)" % (self.id, str(self.tvec[0]), str(np.rad2deg(self.rvec[0])))

<<<<<<< HEAD
class Robot:
=======
class robot:
>>>>>>> 1d80a791a3865431e88396f9ca970d23401cda74
    def __init__(self):
        pass