import pupil_apriltags
import numpy as np
from numpy.typing import ArrayLike
from dataclasses import dataclass
import cv2
from Calibration import Calibration
from math_stuff import Transform3d, Translation3d, Pose3d, Rotation3d
from Tags import FoundTag, KnownTag, field

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

    def find_tags(self, image: ArrayLike) -> list[FoundTag]:
        """Returns a list of all found tags in the frame"""
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        found = self.detector.detect(gray)

        detected = []

        for item in found:
            if item.hamming > 0:
                continue
            image_points = np.array(item.corners)

            success, rotation_vector, translation_vector = cv2.solvePnP(self.object_points, image_points, self.calibration.mtx, self.calibration.dist)
            if success:
                rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
                known_pos = field[item.tag_id]
                """
                  x
                 ___
                |   |
                |   |  y
                |___|
                
                vertical is z
                """
                if known_pos == None:  # TODO replace with index check, this won't catch it
                    continue

                found_tag = FoundTag(known_pos, translation_vector, rotation_matrix)
                detected.append(found_tag)
        return detected