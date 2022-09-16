import entities
import numpy as np
from cv2 import aruco

# global
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100) # aruco dictionary for robots
aruco_dict_static = aruco.getPredefinedDictionary(aruco.DICT_4X4_250) # aruco dictionary for static markers
# field
measurements_count = 100 # count of frames for calibration
static_markers_sked = [100, 101, 102] # list of static marker ids
staticMarker_size = 0.176 # size of static marker [meters]
ms = staticMarker_size
static_poses = (
    (1.0 + ms / 2, 0.081, ms / 2),
    (0.0 + ms / 2, 0.33, ms / 2),
    (0.5 + ms / 2, 0.795,  ms / 2)
)
field_size = (0.8, 0.6) # size of battlefield [meters]

# robots
robots = [entities.Robot("Doshirak"), entities.Robot("Waffle"), entities.Robot("Burger")]
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

robots[2].markers[94] = np.array((
        (0.04, 0.05, 0.072),
        (-0.04, 0.05, 0.072),
        (-0.04, 0.05, 0.008),
        (0.04, 0.05, 0.008)
    ), dtype="float32")
robots[2].markers[95] = np.array((
        (-0.05, 0.04, 0.072),
        (-0.05, -0.04, 0.072),
        (-0.05, -0.04, 0.008),
        (-0.05, 0.04, 0.008)
    ), dtype="float32")
robots[2].markers[96] = np.array((
        (-0.04, -0.05, 0.072),
        (0.04, -0.05, 0.072),
        (0.04, -0.05, 0.008),
        (-0.04, -0.05, 0.008)
    ), dtype="float32")
robots[2].markers[97] = np.array((
        (0.05, -0.04, 0.072),
        (0.05, 0.04, 0.072),
        (0.05, 0.04, 0.008),
        (0.05, -0.04, 0.008)
    ), dtype="float32")