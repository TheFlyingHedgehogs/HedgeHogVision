from dataclasses import dataclass
from quaternions.quaternions import Quaternion


class Translation3d(): pass;

@dataclass
class Translation3d:
    x: float
    y: float
    z: float

    def unaryMinus(self) -> Translation3d:
        return Translation3d(-self.x,-self.y, -self.z)


@dataclass
class Rotation3d:
    q: Quaternion


class Transform3d: pass


@dataclass
class Transform3d:
    translation: Translation3d
    rotation: Rotation3d

    def inverse(self) -> Transform3d:
        return Transform3d(
            self.translation.rotateBy(self.rotation.unaryMinus()),
            self.rotation.unaryMinus
        )