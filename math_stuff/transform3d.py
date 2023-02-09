from dataclasses import dataclass
from math_stuff.rotation3d import Rotation3d
from math_stuff.translation3d import Translation3d
from dashboard import SmartDashboard, NetworkTables
@dataclass
class Transform3d:
    """Describes the transform of and object in 3D space"""
    translation: Translation3d
    """Translation of the transform"""
    rotation: Rotation3d
    """Rotation of the transform"""
    def to_smart_dashboard(self):
        SmartDashboard.putNumber("x", self.translation.x)
        SmartDashboard.putNumber("y", self.translation.y)
        SmartDashboard.putNumber("z", self.translation.z)
        SmartDashboard.putNumber("r", self.rotation.q.x)
        SmartDashboard.putNumber("i", self.rotation.q.y)
        SmartDashboard.putNumber("j", self.rotation.q.z)
        SmartDashboard.putNumber("k", self.rotation.q.w)
        NetworkTables.flush()

    def __add__(self, other):
        return Transform3d(self.translation + other.translation, self.rotation + other.rotation)

    def __truediv__(self, other):
        if type(other) is int:
            return Transform3d(self.translation / other, self.rotation / other)

    def inverse(self):
        """
        :return: The inverse transform
        :rtype: Transform3d
        """
        return Transform3d(
            self.translation.unary_minus().rotate_by(self.rotation.unary_minus()),
            self.rotation.unary_minus()
        )

    def field_distance(self, other) -> float:
        """:returns: the distance between the transform's x and y values
        :param other: Transform to find distance to
        """
        return self.translation.field_distance(other.translation)

    @staticmethod
    def zero():
        """Empty Transform3d instance
        :return: a Transform3d with zeros as all values
        :rtype: Transform3d"""
        return Transform3d(Translation3d.zero(), Rotation3d.zero())
    def __str__(self):
        return f"Transform:\n\t{self.translation},\n\t{self.rotation}"
    @staticmethod
    def average(transforms):
        """:param transforms: The list transforms to be averaged
        :return: A Transform3d in the center of transforms
        :rtype: Transform3d"""
        return_transform = Transform3d.zero()
        for i in transforms:
            return_transform += i
        return_transform = return_transform / len(transforms)
        return return_transform