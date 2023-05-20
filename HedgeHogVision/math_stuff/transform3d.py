from dataclasses import dataclass
from HedgeHogVision.math_stuff.rotation3d import Rotation3d
from HedgeHogVision.math_stuff.translation3d import Translation3d
from HedgeHogVision.SmartDashboard import dashboard
from statistics import stdev


@dataclass
class Transform3d:
    """Describes the transform of and object in 3D space"""
    translation: Translation3d
    """Translation of the transform"""
    rotation: Rotation3d
    """Rotation of the transform"""
    def to_smart_dashboard(self, name, *args):
        if(self.translation.is_zero()): return
        if(self.translation.x < 0 or self.translation.z < 0): return
        if(self.translation.x > 8.2296 or self.translation.z > 16.4592): return
        #print("Put network tables")
        upload = [self.translation.z,
                  self.translation.x,
                  self.translation.y,
                  self.rotation.to_euler_angles()[0]
                  ]
        if(len(args) > 0):
            for i in args: upload.append(i)
        dashboard.SmartDashboard.putNumberArray(name,
                                                upload
                                                )
        #SmartDashboard.putNumber("r", self.rotation.q.x)
        #SmartDashboard.putNumber("i", self.rotation.q.y)
        #SmartDashboard.putNumber("j", self.rotation.q.z)
        #SmartDashboard.putNumber("k", self.rotation.q.w)
        dashboard.NetworkTables.flush()

    def __add__(self, other):
        return Transform3d(self.translation + other.translation, self.rotation + other.rotation)

    def __truediv__(self, other):
        if type(other) is int or float:
            return Transform3d(self.translation / other, self.rotation / other)
    def __sub__(self, other):
        return Transform3d(self.translation - other.translation, self.rotation - other.rotation)
    def __mul__(self, other):
        return Transform3d(self.translation * other, self.rotation)

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
        """:returns: the distance between the transform's x and y and z values
        :param other: Transform to find distance to
        """
        return self.translation.field_distance(other.translation)
    def abs(self):
        return Transform3d(self.translation.abs(),Rotation3d.zero())

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
        if(len(transforms) == 0): return Transform3d.zero()

        return_transform = Transform3d.zero()
        for i in transforms:
            return_transform += i
        return_transform = return_transform / len(transforms)
        return return_transform
    @staticmethod
    def averageDistanceTo(transforms, center):
        """:param transforms: The list transforms to be averaged
        :return: A Transform3d in the center of transforms
        :rtype: Transform3d"""
        if(len(transforms) == 0): return Transform3d.zero()
        return_transform = Transform3d.zero()
        for i in transforms:
            return_transform += (i - center).abs()
        return_transform = return_transform / len(transforms)
        return return_transform
    @staticmethod
    def standardDev(center, points) -> float:
        """

        :param center: Average of all points
        :return: average distance from center
        """
        distances = []
        for point in points:
            distances.append(point.field_distance(center))
        return stdev(distances)
    def transform_by(self, transformation):
        """
        :param transformation: How to move the Transform3d
        :return: Transform3d moved by given Transform3d
        :rtype: Transform3d"""
        new_translation = self.translation + transformation.translation.rotate_by(self.rotation)
        new_rotation = Rotation3d(self.rotation.q * transformation.rotation.q)
        return Transform3d(new_translation, new_rotation)