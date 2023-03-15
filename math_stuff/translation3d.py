from dataclasses import dataclass
from pyquaternion import Quaternion
from numpy.typing import ArrayLike
from math_stuff.rotation3d import Rotation3d
from math import sqrt
@dataclass
class Translation3d:
    """Describes the position of an object in 3D space"""
    x: float
    """x distance from 0, 0, 0"""
    y: float
    """y distance from 0, 0, 0"""
    z: float
    """z distance from 0, 0, 0"""
    @staticmethod
    def from_matrix(matrix: ArrayLike):
        """:return: Translation3d from a given numpy matrix
        :rtype: Translation3d"""
        x = matrix[0][0]
        y = matrix[1][0]
        z = matrix[2][0]
        return Translation3d(x, y, z)
    def unary_minus(self):
        """:return: Translation3d with all values flipped
        :rtype: Translation3d"""
        return Translation3d(-self.x, -self.y, -self.z)
    def abs(self):
        return Translation3d(abs(self.x), abs(self.y), abs(self.z))

    def rotate_by(self, other: Rotation3d):
        """
        :param other: the Rotation3d to rotate the translation by
        :return: Translation3d rotated by other
        :rtype: Translation3d"""
        p = Quaternion(0.0, self.x, self.y, self.z)
        qprime = other.q * p * other.q.conjugate
        return Translation3d(qprime.x, qprime.y, qprime.z)

    def __add__(self, other):
        return Translation3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __truediv__(self, other):
        if type(other) is int:
            return Translation3d(self.x / other, self.y / other, self.z / other)
    def __sub__(self, other):
        return Translation3d(self.x - other.x, self.y - other.y, self.z - other.z)
    def __str__(self):
        return f"Translation: (x: {self.x},y: {self.y}, z: {self.z})"

    @staticmethod
    def zero():
        """Empty Translation3d instance
        :return: A Translation3d with zeros as all values
        :rtype: Translation3d"""
        return Translation3d(0.0, 0.0, 0.0)
    def is_zero(self) -> bool:
        return self.x == 0 and self.y ==0 and self.z == 0
    def field_distance(self, other) -> float:
        """:return: The distance between self and "other"
        :param: other the second translation to find the distance to"""
        return sqrt((self.x-other.x)**2 + (self.z-other.z)**2 + (self.y - other.y)**2)
    def field_distance2D(self,other):
        return sqrt((self.x-other.x)**2 + (self.z-other.z)**2)
