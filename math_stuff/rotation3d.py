from dataclasses import dataclass
from pyquaternion import Quaternion
from numpy.typing import ArrayLike
import math
@dataclass
class Rotation3d:
    """Describes the rotation of an object in 3D space"""
    q: Quaternion
    def to_euler_angles(self):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        :author: automaticaddison
        """
        t0 = +2.0 * (self.q.w * self.q.x + self.q.y * self.q.z)
        t1 = +1.0 - 2.0 * (self.q.x * self.q.x + self.q.y * self.q.y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (self.q.w * self.q.y - self.q.z * self.q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (self.q.w * self.q.z + self.q.x * self.q.y)
        t4 = +1.0 - 2.0 * (self.q.y * self.q.y + self.q.z * self.q.z)
        yaw_z = math.atan2(t3, t4)

        return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z) # in radians

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
    def __str__(self):
        x, y, z = self.to_euler_angles()
        return f"Rotation: (x: {x}, y: {y}, z: {z})"

    @staticmethod
    def zero():
        """Empty Pose3d instance
        :return: Rotation3d with all values as zero.
        :rtype: Rotation3d"""
        return Rotation3d(Quaternion(axis=[1.0, 0.0, 0.0], radians=0.0))
