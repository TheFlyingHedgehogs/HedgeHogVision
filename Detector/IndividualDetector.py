from Detector.Detector import Detector
import cv2
import numpy as np
from Tags import FoundTag, field, MegaTag
from math_stuff.math_stuff import Transform3d
from dashboard import SmartDashboard
from math_stuff.rotation3d import Rotation3d
from math_stuff.translation3d import Translation3d
"""
                       x
                 ____________
                |            |  y
                |____________|

                vertical is z
            """
class IndividualDetector(Detector):
    def create_tags(self, tags) -> list[list[FoundTag]]:
        """Returns a list of all found tag pairs in the frame"""
        detected = []
        for item in tags:
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

            success, rotation_vectors, translation_vectors, _ = cv2.solvePnPGeneric(self.object_points, image_points, self.calibration.mtx, self.calibration.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not success: continue

            known_tag = field[item.tag_id]

            to_append = []
            for (r, t) in zip(rotation_vectors,translation_vectors):
                rotation_matrix = cv2.Rodrigues(r)[0]
                found_tag = FoundTag(known_tag, t, rotation_matrix,item.tag_id)
                to_append.append(found_tag)
            detected.append(to_append)
        return detected
