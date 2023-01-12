from dataclasses import dataclass
from pyquaternion import Quaternion
from numpy.typing import ArrayLike


@dataclass
class Rotation3d:
    q: Quaternion

    def unary_minus(self):
        return Rotation3d(self.q.conjugate())

    @staticmethod
    def from_matrix(matrix: ArrayLike):
        return Rotation3d(Quaternion(matrix=matrix))

    def __add__(self, other):
        return Rotation3d(self.q * other.q)

    @staticmethod
    def zero():
        return Rotation3d(Quaternion(radians=[0.0, 0.0, 0.0]))


@dataclass
class Translation3d:
    x: float
    y: float
    z: float
    @staticmethod
    def from_matrix(Matrix: ArrayLike):
        x = Matrix[0]
        y = Matrix[1]
        z = Matrix[2]
        return Translation3d(x,y,z)
    def unary_minus(self):
        return Translation3d(-self.x, -self.y, -self.z)

    def rotate_by(self, other: Rotation3d):
        p = Quaternion(0.0, self.x, self.y, self.z)
        qprime = other.q * p * other.q.conjugate()
        return Translation3d(qprime.real, qprime.i, qprime.j)

    def __add__(self, other):
        return Translation3d(self.x + other.x, self.y + other.y, self.z + other.z)

    @staticmethod
    def zero():
        return Translation3d(0.0, 0.0, 0.0)


@dataclass
class Pose3d:
    translation: Translation3d
    rotation: Rotation3d

    def transform_by(self, transformation):
        new_translation = self.translation + transformation.translation.rotate_by(self.rotation)
        new_rotation = transformation.rotation + self.rotation
        return Pose3d(new_translation, new_rotation)

    @staticmethod
    def zero():
        return Pose3d(Translation3d.zero(), Rotation3d.zero())


@dataclass
class Transform3d:
    translation: Translation3d
    rotation: Rotation3d

    def inverse(self):
        return Transform3d(
            self.translation.rotate_by(self.rotation.unary_minus()),
            self.rotation.unary_minus()
        )

    @staticmethod
    def zero():
        return Transform3d(Translation3d.zero(), Rotation3d.zero())
