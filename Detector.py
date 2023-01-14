import cv2
import numpy as np
import pupil_apriltags
from numpy.typing import ArrayLike
from Calibration import Calibration
from Tags import FoundTag, field
from math_stuff.math_stuff import Transform3d

class Detector:
    """Used to find Apriltags in an image and return a position on the field"""
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
                if known_pos is None:  # TODO replace with index check, this won't catch it
                    continue

                found_tag = FoundTag(known_pos, translation_vector, rotation_matrix)
                detected.append(found_tag)
        return detected

    def get_world_pos_from_image(self, img):
        tags = self.find_tags(img)
        transforms = []
        for i in tags:
            robo_location = i.get_robot_location()
            transforms.append(robo_location)
        position = Transform3d.average(transforms)
        return position
