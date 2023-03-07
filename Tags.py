from dataclasses import dataclass

import numpy

from math_stuff.math_stuff import Transform3d, Translation3d, Pose3d, Rotation3d
from pyquaternion import Quaternion
from numpy.typing import ArrayLike
import math
from constants import _robotToCamera


@dataclass
class KnownTag:
    """Contains the position and rotation of the tag on the field"""

    @staticmethod
    def from_inches(x_inches: float, z_inches: float, y_inches: float, rotation_degrees: float):
        return KnownTag(x_inches * 0.0254, y_inches * 0.0254, z_inches * 0.0254, rotation_degrees)

    def __init__(self, x: float, y: float, z: float, rotation_degrees: float):
        self.x: float = x
        """X position of the tag relative to a corner of the field in meters"""
        self.y: float = y
        """Y position of the tag relative to a corner of the field in meters"""
        self.z: float = z
        """Z position of the tag relative to a corner of the field in meters"""
        self.rotation: float = math.radians(rotation_degrees)
        self.flipped = None
        """If the tag is facing toward 0,0"""
        if rotation_degrees == 0: self.flipped = False
        else: self.flipped = True
        """Rotation of the tag relative to the center of the field in radians."""
        self.pose = Pose3d(
            Translation3d(x, y, z),
            # Rotation3d.zero()
            # Translation3d.zero(),
            Rotation3d(Quaternion(axis=[1.0, 0.0, 0.0], radians=self.rotation))
        )


class MegaTag(KnownTag):
    def __init__(self, x: float, y: float, z: float, rotation_degrees: float):
        KnownTag.__init__(self, x, y, z, rotation_degrees)


class FoundTag:
    def __get_robot_location(self):
        """

        :return: Robots real world position
        :rtype: Pose3d
        """
        object_to_camera = self.tag_transform.inverse()
        camera_to_robot = _robotToCamera.inverse()
        return self.parent_tag.pose.transform_by(object_to_camera).transform_by(camera_to_robot)
    def __get_guess_location(self):
        """
        :return: Robots real world position
        :rtype: Pose3d
        """
        object_to_camera = self.tag_transform.inverse()
        camera_to_robot = _robotToCamera.inverse()
        return Pose3d.zero().transform_by(object_to_camera).transform_by(camera_to_robot)

    def __init__(self, parent_tag: KnownTag, translation: ArrayLike, rotation: ArrayLike, id: int = 0):
        self.id = id
        self.parent_tag: KnownTag = parent_tag
        """The ID of the apriltag"""
        translation3d: Translation3d = Translation3d.from_matrix(translation)
        """Translation of the camera from the apriltag"""
        rotation3d: Rotation3d = Rotation3d.from_matrix(rotation)
        """Rotation matrix of the apriltag from the matrix"""
        self.tag_transform: Transform3d = Transform3d(translation3d, rotation3d)
        self.robot_position = self.__get_robot_location()
        self.distance = self.__get_guess_location()
        self.first_guess = numpy.matmul(rotation, translation)

    def __str__(self):
        return f"Tag: {self.tag_transform}"

field = (
    KnownTag.from_inches(0.0, 0.0, 0.0, 180),           # 0
    KnownTag.from_inches(42.19, 610.77, 18.22, 180),    # 1
    KnownTag.from_inches(108.19, 610.77, 18.22, 180),   # 2
    KnownTag.from_inches(147.19, 610.77, 18.22, 180),   # 3
    KnownTag.from_inches(265.74, 636.96, 27.38, 180),   # 4
    KnownTag.from_inches(265.74,  14.25, 27.38, 0),     # 5
    KnownTag.from_inches(147.19,  40.45, 18.22, 0),     # 6
    KnownTag.from_inches(108.19,  40.45, 18.22, 0),     # 7
    KnownTag.from_inches(42.19,  40.45, 18.22, 0)       # 8
)
"""field = (
    None,
    KnownTag(-1, 0, 0, 180),
    KnownTag(1, 0, 0, 180),
    KnownTag(1.91, 0, 0, 180),
    KnownTag(-1, 0, 0, 180),
    KnownTag(1, 0, 0, 180),
    KnownTag(3, 0, 0, 180),
    KnownTag(0, 0, 0, 180),
    KnownTag(0.87, 0, 0, 180),
)"""
"""field = (
    None,
    KnownTag.from_inches(132, -99, 0, 0),
    KnownTag.from_inches(31+132, -99, 0, 0),
    KnownTag.from_inches(28+31+132, -99, 0, 0),
)"""
