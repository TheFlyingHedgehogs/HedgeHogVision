from dataclasses import dataclass
from quaternions.quaternions import Quaternion


@dataclass
class Translation3d:
    x: float
    y: float
    z: float


@dataclass
class Rotation3d:
    q: Quaternion


@dataclass
class Transform3d:
    translation: Translation3d
    rotation: Rotation3d
