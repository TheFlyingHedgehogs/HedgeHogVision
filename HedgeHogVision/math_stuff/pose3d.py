from dataclasses import dataclass
from HedgeHogVision.math_stuff.rotation3d import Rotation3d
from HedgeHogVision.math_stuff.translation3d import Translation3d
from HedgeHogVision.math_stuff.transform3d import Transform3d
@dataclass
class Pose3d(Transform3d):
    """Describes a 3d pose"""
    translation: Translation3d
    rotation: Rotation3d

    def transform_by(self, transformation: Transform3d):
        """
        :param transformation: How to move the Pose3d
        :return: Pose3d moved by given Transform3d
        :rtype: Pose3d"""
        new_translation = self.translation + transformation.translation.rotate_by(self.rotation)
        new_rotation = Rotation3d(self.rotation.q * transformation.rotation.q)
        return Pose3d(new_translation, new_rotation)

    def field_distance(self,other):
        """Gets the distance from other in 2D"""
        return self.translation.field_distance(other.translation)
    def field_distance2D(self, other):
        return  self.translation.field_distance2D(other.translation)
    def __repr__(self):
        return f"Pose:\n\t{self.translation},\n\t{self.rotation}"

    @staticmethod
    def zero():
        """Empty Pose3d instance
        :return: Pose3d with all values as zero.
        :rtype: Pose3d"""
        return Pose3d(Translation3d.zero(), Rotation3d.zero())
