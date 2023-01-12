from dataclasses import dataclass
from math_stuff.rotation3d import Rotation3d
from math_stuff.translation3d import Translation3d
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
