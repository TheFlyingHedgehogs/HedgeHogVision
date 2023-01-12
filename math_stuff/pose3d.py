from dataclasses import dataclass
from pyquaternion import Quaternion
from numpy.typing import ArrayLike
from math_stuff.rotation3d import Rotation3d
from math_stuff.translation3d import Translation3d
from math_stuff.transform3d import Transform3d
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
