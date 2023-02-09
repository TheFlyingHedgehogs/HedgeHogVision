import cv2
import numpy
import numpy as np
import pupil_apriltags
from numpy.typing import ArrayLike
from Calibration import Calibration
from Tags import FoundTag, field
from math_stuff.math_stuff import Transform3d
from dashboard import SmartDashboard
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
    def update(self):
        self.roborioPosition = Translation3d(Translation3d.getValue("RobotX"), 0, Translation3d.getValue("RobotY"))
    def __init__(self, calibration: Calibration, tag_width_m: float = 0.1524):
        self.lastKnownPosition: Transform3d = None
        self.roborioPosition: Translation3d = None
        self.listOfLastPos = []
        self.time_since_last_update = 0

        self.calibration = calibration
        self.detector = pupil_apriltags.Detector(families="tag16h5", nthreads=4)  # TODO test thread count

        tag_half = tag_width_m / 2
        self.object_points = np.array([
            [-tag_half,  tag_half, 0.0],
            [ tag_half,  tag_half, 0.0],
            [ tag_half, -tag_half, 0.0],
            [-tag_half, -tag_half, 0.0]
        ], dtype=np.float64)

    def find_tags(self, image: ArrayLike) -> list[list[FoundTag]]:
        """Returns a list of all found tag pairs in the frame"""
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

            success, rotation_vectors, translation_vectors, _ = cv2.solvePnPGeneric(self.object_points, image_points, self.calibration.mtx, self.calibration.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not success: continue
            if item.tag_id >= len(field) or item.tag_id == 0: continue

            known_tag = field[item.tag_id]
            rotation_vector, translation_vector = None, None

            # If first translation vector is behind tag, and tag is flipped, choose the first option
            #vectori0 = Rotation3d.from_matrix(cv2.Rodrigues(rvecs[0])[0]).q.axis
            to_append = []
            for (r, t) in zip(rotation_vectors,translation_vectors):
                rotation_matrix = cv2.Rodrigues(r)[0]
                found_tag = FoundTag(known_tag, t, rotation_matrix)
                to_append.append(found_tag)
            detected.append(to_append)
            """
                       x
                 ____________
                |            |  y
                |____________|

                vertical is z
            """
        return detected

    def trimmed_tags(self, image: ArrayLike):
        """Returns a list of tags, trimming the incorrect tags from the pairs"""
        tags = self.find_tags(image)
        if len(tags) == 0: return []
        if len(tags) == 1:
            if(self.roborioPosition != None):
                return [tags[0][self.roborioPosition.field_distance(tags[0][0].robot_position.translation)]]
            if(self.lastKnownPosition == None): return [tags[0][tags[0][0].robot_position.translation.y < 0]]
            return [tags[0][self.lastKnownPosition.field_distance(tags[0][0].robot_position) > self.lastKnownPosition.field_distance(tags[0][1].robot_position)]]
        for i in tags:
            print(i[0].robot_position)
            print(i[1].robot_position)
        first_tag_pos = tags[0][0].robot_position
        first_total_error = 0
        first_tags = []

        second_tag_pos = tags[0][1].robot_position
        second_total_error = 0
        second_tags = []
        for [a,b] in tags[1:]:
            a_dist_first = first_tag_pos.field_distance(a.robot_position)
            b_dist_first = first_tag_pos.field_distance(b.robot_position)
            first_total_error += min(a_dist_first,b_dist_first)
            first_tags.append(tags[0][a_dist_first > b_dist_first])

            a_dist_second = second_tag_pos.field_distance(a.robot_position)
            b_dist_second = second_tag_pos.field_distance(b.robot_position)
            second_total_error += min(a_dist_second, b_dist_second)
            first_tags.append(tags[0][a_dist_second > b_dist_second])
        if first_total_error < second_total_error: return first_tags
        return second_tags
    def get_world_pos_from_image(self, img):
        """Used to get real world position from apriltags on the feild.
        :return: The field position of the bot
        :rtype: Transform3d
        """
        tags = self.trimmed_tags(img)
        if len(tags) == 0:
            return Transform3d.zero()
        transforms = []
        for i in tags:
            transforms.append(i.robot_position)
        position = Transform3d.average(transforms)
        self.lastKnownPosition = position
        self.listOfLastPos.append(position)
        return position

    def get_world_pos_from_image_debug(self, img):
        """Used to get real world position from apriltags on the feild.
        :return: The field position of the bot
        :rtype: Transform3d
        """
        tags = self.trimmed_tags(img)
        if len(tags) == 0:
            return Transform3d.zero(), 0
        transforms = []
        for i in tags:
            transforms.append(i.robot_position)
        position = Transform3d.average(transforms)
        self.lastKnownPosition = position
        self.listOfLastPos.append(position)
        return position, len(tags)


    """def world_poses(self, img):
        tags = self.find_tags(img)
        if len(tags) == 0:
            return []
        transforms = []
        for i in tags:
            robo_location = i.get_robot_location()
            transforms.append((robo_location, i.tag_transform.translation, i.parent_tag.pose, i.parent_tag))
        # position = Transform3d.average(transforms)
        # return position
        return transforms"""
