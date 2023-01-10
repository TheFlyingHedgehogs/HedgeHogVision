from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike
import pupil_apriltags
import cv2
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
    id: int
    translation: ArrayLike
    rotation: ArrayLike


@dataclass
class KnownTag:
    x: float
    y: float
    z: float
    rotation: float


def t(x_in: float, y_in: float, z_in: float, rotation: float) -> KnownTag:
    return KnownTag(x_in * 0.0254, y_in * 0.0254, z_in * 0.0254, rotation)

field = {
    1: t( 42.19, 610.77, 18.22, 180),
    2: t(108.19, 610.77, 18.22, 180),
    3: t(147.19, 610.77, 18.22, 180),
    4: t(265.74, 636.96, 27.38, 180),
    5: t(265.74,  14.25, 27.38, 0),
    6: t(147.19,  40.45, 18.22, 0),
    7: t(108.19,  40.45, 18.22, 0),
    8: t( 42.19,  40.45, 18.22, 0)
}


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
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        found = self.detector.detect(gray)

        detected = []

        for item in found:
            if item.hamming > 0:
                continue
            image_points = np.array(item.corners)

            ret, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.calibration.mtx, self.calibration.dist)
            if ret:
                r_matrix = cv2.Rodrigues(rvec)[0]

                tag_coords = np.matmul(r_matrix, tvec)

                known_pos = field[item.tag_id]
                # print(item.tag_id)
                # print(field)
                # print(field[item.tag_id])

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
                print(tag_coords)
                # print(known_pos)
                print(tvec)
                world_coords = (known_pos.x + tag_coords[0][0], known_pos.y + tag_coords[2][0], known_pos.z + tag_coords[1][0])
                # print(world_coords)


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
