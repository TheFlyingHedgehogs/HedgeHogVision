import numpy
from HedgeHogVision.math_stuff.math_stuff import Transform3d, Translation3d, Pose3d, Rotation3d
from pyquaternion import Quaternion
import math
class KnownTag:
    """Contains the position and rotation of the tag on the field"""

    @staticmethod
    def from_inches(id, x_inches: float, z_inches: float, y_inches: float, rotation_degrees: float):
        return KnownTag(id, x_inches * 0.0254, y_inches * 0.0254, z_inches * 0.0254, rotation_degrees)

    def __init__(self, id, x: float, y: float, z: float, rotation_degrees: float, tag_witdth: float = 0.1524):
        self.id = id
        self.x: float = x
        """X position of the tag relative to a corner of the field in meters"""
        self.y: float = y
        """Y position of the tag relative to a corner of the field in meters"""
        self.z: float = z
        """The width of the tag in meters"""
        self.tag_witdth = tag_witdth
        """Z position of the tag relative to a corner of the field in meters"""
        self.rotation: float = math.radians(rotation_degrees)
        self.rotationDegrees: float = rotation_degrees

        tag_half = tag_witdth / 2
        self.object_points = numpy.array([
            [-tag_half,  tag_half, 0.0],
            [ tag_half,  tag_half, 0.0],
            [ tag_half, -tag_half, 0.0],
            [-tag_half, -tag_half, 0.0]
        ], dtype=numpy.float64)

        """Rotation of the tag relative to the center of the field in radians."""
        self.pose = Pose3d(
            Translation3d(x, y, z),
            Rotation3d(Quaternion(axis=[1.0, 0.0, 0.0], radians=self.rotation))
        )