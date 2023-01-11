from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike
import pupil_apriltags
import cv2
import math
from pprint import pprint

@dataclass
class Calibration:
    mtx: ArrayLike
    dist: ArrayLike


class PinholeCalibration(Calibration):
    pass


def perfect_camera(focal_length_mm, horizontal_sensor_size_mm, resolution) -> PinholeCalibration:
    focal_length = (resolution[0] / horizontal_sensor_size_mm) * focal_length_mm

    mtx = np.array([[focal_length,          0.0, resolution[0] / 2],
                    [0.0,          focal_length, resolution[1] / 2],
                    [0.0,                   0.0, 1.0              ]])

    dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

    return PinholeCalibration(mtx, dist)

class FoundTag:
    def __init__(self, parent_tag: int, translation: ArrayLike, rotation: ArrayLike):
        parent_tag: KnownTag = parent_tag
        """The ID of the apriltag"""
        translation: ArrayLike = translation
        """Translation of the camera from the apriltag"""
        rotation: ArrayLike = rotation
        """Rotation matrix of the apriltag from the matrix"""


@dataclass
class KnownTag:
    """Contains the position and rotation of the tag on the field"""
    def __init__(self, x_inches: float, y_inches: float, z_inches: float, rotation_degrees: float):
        self.x: float = x_inches * 0.0254
        """X position of the tag relative to a corner of the field in meters"""
        self.y: float = y_inches * 0.0254
        """Y position of the tag relative to a corner of the field in meters"""
        self.z: float = z_inches * 0.0254
        """Z position of the tag relative to a corner of the field in meters"""
        self.rotation: float = math.radians(rotation_degrees)
        """Rotation of the tag relative to the center of the field in radians."""


"""def t(x_in: float, y_in: float, z_in: float, rotation: float) -> KnownTag:
    return KnownTag(x_in * 0.0254, y_in * 0.0254, z_in * 0.0254, rotation)
"""
field = (
    None,                                   #0
    KnownTag(42.19, 610.77, 18.22, 180),    #1
    KnownTag(108.19, 610.77, 18.22, 180),   #2
    KnownTag(147.19, 610.77, 18.22, 180),   #3
    KnownTag(265.74, 636.96, 27.38, 180),   #4
    KnownTag(265.74,  14.25, 27.38, 0),     #5
    KnownTag(147.19,  40.45, 18.22, 0),     #6
    KnownTag(108.19,  40.45, 18.22, 0),     #7
    KnownTag(42.19,  40.45, 18.22, 0)       #8
)


class Detector:
    def __init__(self, calibration: Calibration, tag_width_m: float = 0.1524):
        self.calibration = calibration
        self.detector = pupil_apriltags.Detector(families="tag16h5")  # TODO test thread count

        tag_half = tag_width_m / 2
        self.object_points = np.array([
            [-tag_half, -tag_half, 0.0],
            [tag_half, -tag_half, 0.0],
            [tag_half, tag_half, 0.0],
            [-tag_half, tag_half, 0.0]
        ], dtype=np.float64)
    
    def find_tags(self, image: ArrayLike) -> list[FoundTag()]:
        """Returns a list of all found tags in the frame"""
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        found = self.detector.detect(gray)

        detected = []

        for item in found:
            if item.hamming > 0:
                continue
            image_points = np.array(item.corners)

            success, rotation_vector, translation_vector = cv2.solvePnP(self.object_points, image_points, self.calibration.mtx, self.calibration.dist)
            if success:
                rotation_matrix = cv2.Rodrigues(rotation_vector)[0]

                tag_coords = np.matmul(rotation_matrix, translation_vector)

                known_pos = field[item.tag_id]
                """
                  x
                 ___
                |   |
                |   |  y
                |___|
                
                vertical is z
                """
                if known_pos == None:
                    continue
                detected.append(FoundTag(known_pos, translation_vector, rotation_matrix))
                print(tag_coords)
                # print(known_pos)
                print(translation_vector)
                world_coords = (known_pos.x + tag_coords[0][0], known_pos.y + tag_coords[2][0], known_pos.z + tag_coords[1][0])
                # print(world_coords)
        return detected

img = cv2.imread("/tmp/0251.png")
# img = cv2.imread("/home/foo/synced/2023-field/angles/moved.png")
# img = cv2.imread("/tmp/Untitled.png")
# img = cv2.imread("/home/foo/synced/2023-field/tags/tag16_05_00005.png")
calibration = perfect_camera(50, 36, (1920, 1080))

# d = pupil_apriltags.Detector(families="tag16h5")
# d = dt_apriltags.Detector(families='tag16h5')
# print(d.detect(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)))

detector = Detector(calibration)

detector.find_tags(img)
