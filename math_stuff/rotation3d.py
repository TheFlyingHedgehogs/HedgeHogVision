from dataclasses import dataclass
from pyquaternion import Quaternion
from numpy.typing import ArrayLike

@dataclass
class Rotation3d:
    """Describes the rotation of an object in 3D space"""
    q: Quaternion

    def unary_minus(self):
        """:return: Rotation3d with all Quaternions values flipped
        :rtype: Rotation3d"""
        return Rotation3d(self.q.conjugate)

    @staticmethod
    def from_matrix(matrix: ArrayLike):
        """:return: Rotation3d from a rotation matrix
        :rtype: Rotation3d"""
        return Rotation3d(Quaternion(matrix=matrix))

    def __add__(self, other):
        return Rotation3d(self.q * other.q)

    def __truediv__(self, other):
        if type(other) is int:
            return Rotation3d(self.q / other)

    @staticmethod
    def zero():
        """Empty Pose3d instance
        :return: Rotation3d with all values as zero.
        :rtype: Rotation3d"""
        return Rotation3d(Quaternion(axis=[1.0, 0.0, 0.0], radians=0.0))
