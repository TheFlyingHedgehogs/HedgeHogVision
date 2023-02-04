import cv2
import numpy as np
import pupil_apriltags
from numpy.typing import ArrayLike
from Calibration import Calibration
from Tags import FoundTag, field
from math_stuff.math_stuff import Transform3d

from math_stuff.rotation3d import Rotation3d
from math_stuff.translation3d import Translation3d

"""
b    g
+---+
|   |
+---+
p    r
"""

class Detector:
    """Used to find Apriltags in an image and return a position on the field"""
    def __init__(self, calibration: Calibration, tag_width_m: float = 0.1524):
        self.calibration = calibration
        self.detector = pupil_apriltags.Detector(families="tag16h5", nthreads=4)  # TODO test thread count

        tag_half = tag_width_m / 2
        self.object_points = np.array([
            [-tag_half,  tag_half, 0.0],
            [ tag_half,  tag_half, 0.0],
            [ tag_half, -tag_half, 0.0],
            [-tag_half, -tag_half, 0.0]
        ], dtype=np.float64)

    def find_tags(self, image: ArrayLike) -> list[FoundTag]:
        """Returns a list of all found tags in the frame"""
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        found = self.detector.detect(gray)

        detected = []

        for item in found:
            if item.hamming > 0 or item.decision_margin < 0.8:
                continue
            pts = np.array(item.corners)
            image_points = item.corners
            """
            expected

            1   2
            +---+
            |   |
            +---+
            4   3

            4   3
            +---+
            |   |
            +---+
            1   2
            """

            def swap(a, b):
                image_points[b] = pts[a]
                image_points[a] = pts[b]

            swap(0, 3)
            swap(1, 2)
            swap(2, 1)
            swap(3, 0)
            c = image_points

            cv2.drawMarker(image, ((int(c[0][0]), int(c[0][1]))), (255, 0, 0))
            cv2.drawMarker(image, ((int(c[1][0]), int(c[1][1]))), (0, 255, 0))
            cv2.drawMarker(image, ((int(c[2][0]), int(c[2][1]))), (0, 0, 255))
            cv2.drawMarker(image, ((int(c[3][0]), int(c[3][1]))), (255, 0, 255))

            # success, rotation_vector, translation_vector = cv2.solvePnP(self.object_points, image_points, self.calibration.mtx, self.calibration.dist)

            success, rvecs, tvecs, _ = cv2.solvePnPGeneric(self.object_points, image_points, self.calibration.mtx, self.calibration.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not success: continue
            if item.tag_id >= len(field) or item.tag_id == 0: continue

            known_tag = field[item.tag_id]
            rotation_vector, translation_vector = None, None

            # If first translation vector is behind tag, and tag is flipped, choose the first option
            vectori0 = Rotation3d.from_matrix(cv2.Rodrigues(rvecs[0])[0]).q.vector
            """Index of the negative rotation vector"""
            i = 0
            if vectori0[2] < 0:
                i = 0
            else:
                i = 1
            if known_tag.flipped:
                translation_vector = tvecs[not i]
                rotation_vector = rvecs[not i]
            else:
                translation_vector = tvecs[i]
                rotation_vector = rvecs[i]


            rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
            """
                       x
                 ____________
                |            |  y
                |____________|

                vertical is z
            """

            found_tag = FoundTag(known_tag, translation_vector, rotation_matrix)


            detected.append(found_tag)
        return detected

    def get_world_pos_from_image(self, img):
        tags = self.find_tags(img)
        if len(tags) == 0:
            return Transform3d.zero()
        transforms = []
        for i in tags:
            robo_location = i.get_robot_location()
            transforms.append(robo_location)
        position = Transform3d.average(transforms)
        return position

    def world_poses(self, img):
        tags = self.find_tags(img)
        if len(tags) == 0:
            return []
        transforms = []
        for i in tags:
            robo_location = i.get_robot_location()
            transforms.append((robo_location, i.tag_transform.translation, i.parent_tag.pose, i.parent_tag))
        # position = Transform3d.average(transforms)
        # return position
        return transforms
