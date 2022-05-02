import entities
import numpy as np
from cv2 import aruco

# global
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# field
measurements_count = 100 # count of frames for calibration
static_markers_sked = [10, 11, 12] # list of static marker ids
staticMarker_size = 0.1 # size of static marker [meters]
ms = staticMarker_size
static_poses = (
    (0.812 + ms / 2, 0.027 + ms / 2, 0.0),
    (0.038 + ms / 2, 0.284 + ms / 2, 0.0),
    (0.637 + ms / 2, 0.405 + ms / 2, 0.0)
)
field_size = (0.8, 0.6) # size of battlefield [meters]

# robots
robots = [entities.Robot("Doshirak"), entities.Robot("Cube")]
robots[0].markers[25] = np.array((
        (0.056, 0.102, 0.153),
        (-0.056, 0.102, 0.153),
        (-0.056, 0.102, 0.064),
        (0.056, 0.102, 0.064)
    ), dtype="float32")
robots[0].markers[14] = np.array((
        (-0.135, -0.057, 0.047),
        (-0.135, 0.043, 0.047),
        (-0.135, 0.043, 0.147),
        (-0.135, -0.057, 0.147)
    ), dtype="float32")

robots[1].markers[90] = np.array((
        (0.04, 0.05, 0.072),
        (-0.04, 0.05, 0.072),
        (-0.04, 0.05, 0.008),
        (0.04, 0.05, 0.008)
    ), dtype="float32")
robots[1].markers[91] = np.array((
        (-0.05, 0.04, 0.072),
        (-0.05, -0.04, 0.072),
        (-0.05, -0.04, 0.008),
        (-0.05, 0.04, 0.008)
    ), dtype="float32")
robots[1].markers[92] = np.array((
        (-0.04, -0.05, 0.072),
        (0.04, -0.05, 0.072),
        (0.04, -0.05, 0.008),
        (-0.04, -0.05, 0.008)
    ), dtype="float32")
robots[1].markers[93] = np.array((
        (0.05, -0.04, 0.072),
        (0.05, 0.04, 0.072),
        (0.05, 0.04, 0.008),
        (0.05, -0.04, 0.008)
    ), dtype="float32")