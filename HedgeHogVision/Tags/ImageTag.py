from HedgeHogVision.math_stuff.math_stuff import Transform3d, Translation3d, Pose3d, Rotation3d
from numpy.typing import ArrayLike
from .RealWorldTag import KnownTag
import numpy

class FoundTag:
    def __get_robot_location(self) -> Pose3d:
        """
        :return: Robots real world position
        :rtype: Pose3d
        """
        object_to_camera = self.tag_transform.inverse()
        return self.parent_tag.pose.transform_by(object_to_camera)

    def __init__(self, parent_tag: KnownTag, translation: ArrayLike, rotation: ArrayLike, id: int = 0):
        self.id = id
        """The ID of the apriltag"""
        self.parent_tag: KnownTag = parent_tag
        translation3d: Translation3d = Translation3d.from_matrix(translation)
        """Translation of the camera from the apriltag"""
        rotation3d: Rotation3d = Rotation3d.from_matrix(rotation)
        """Rotation matrix of the apriltag from the matrix"""
        self.tag_transform: Transform3d = Transform3d(translation3d, rotation3d)
        self.robot_position = self.__get_robot_location()
        self.first_guess = numpy.matmul(rotation, translation)

    def __str__(self):
        return f"Tag: {self.tag_transform}"

    def info(self):
        print(f"Pos: {self.robot_position},  \n"
              f"Transform: {self.tag_transform}, \n"
              f"ID: {self.id}")
