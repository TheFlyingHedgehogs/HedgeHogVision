from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike


@dataclass
class Calibration:
    mtx: ArrayLike
    dist: ArrayLike


class PinholeCalibration(Calibration):
    pass


def perfect_camera(focal_length_mm, horizontal_sensor_size_mm, resolution) -> PinholeCalibration:
    focal_length = (resolution[0] / horizontal_sensor_size_mm) * focal_length_mm

    mtx = np.array([[focal_length, 0.0, resolution[0] / 2],
                    [0.0, focal_length, resolution[1] / 2],
                    [0.0, 0.0, 1.0]])

    dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

    return PinholeCalibration(mtx, dist)
