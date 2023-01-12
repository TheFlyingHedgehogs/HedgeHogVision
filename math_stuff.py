from dataclasses import dataclass
from pyquaternion import Quaternion
from numpy.typing import ArrayLike


@dataclass
class Rotation3d:
    """Describes the rotation of an object in 3D space"""
    q: Quaternion

    def unary_minus(self):
        return Rotation3d(self.q.conjugate)

    @staticmethod
    def from_matrix(matrix: ArrayLike):
        return Rotation3d(Quaternion(matrix=matrix))

    def __add__(self, other):
        return Rotation3d(self.q * other.q)

    def __truediv__(self, other):
        if type(other) is int:
            return Rotation3d(self.q / other)

    @staticmethod
    def zero():
        return Rotation3d(Quaternion(axis=[1.0, 0.0, 0.0], radians=0.0))


@dataclass
class Translation3d:
    """Describes the position of an object in 3D space"""
    x: float
    y: float
    z: float

    @staticmethod
    def from_matrix(matrix: ArrayLike):
        x = matrix[0][0]
        y = matrix[1][0]
        z = matrix[2][0]
        return Translation3d(x, y, z)

    def unary_minus(self):
        return Translation3d(-self.x, -self.y, -self.z)

    def rotate_by(self, other: Rotation3d):
        p = Quaternion(0.0, self.x, self.y, self.z)
        qprime = other.q * p * other.q.conjugate
        return Translation3d(qprime.x, qprime.y, qprime.z)

    def __add__(self, other):
        return Translation3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __truediv__(self, other):
        if type(other) is int:
            return Translation3d(self.x + other, self.y + other, self.z + other)

    @staticmethod
    def zero():
        return Translation3d(0.0, 0.0, 0.0)


@dataclass
class Transform3d:
    """Describes the transform of and object in 3D space"""
    translation: Translation3d
    """Translation of the transform"""
    rotation: Rotation3d
    """Rotation of the transform"""

    def __add__(self, other):
        return Transform3d(self.translation + other.translation, self.rotation + other.rotation)

    def __truediv__(self, other):
        if type(other) is int:
            return Transform3d(self.translation / other, self.rotation / other)

    def inverse(self):
        return Transform3d(
            self.translation.unary_minus().rotate_by(self.rotation.unary_minus()),
            self.rotation.unary_minus()
        )

    @staticmethod
    def zero():
        return Transform3d(Translation3d.zero(), Rotation3d.zero())

    @staticmethod
    def average(arg):
        return_transform = Transform3d.zero()
        for i in arg:
            return_transform += i
        return_transform = return_transform / len(arg)
        return return_transform


@dataclass
class Pose3d:
    translation: Translation3d
    rotation: Rotation3d

    def transform_by(self, transformation: Transform3d):
        new_translation = self.translation + transformation.translation.rotate_by(self.rotation)
        new_rotation = Rotation3d(self.rotation.q * transformation.rotation.q)
        return Pose3d(new_translation, new_rotation)

    @staticmethod
    def zero():
        return Pose3d(Translation3d.zero(), Rotation3d.zero())
