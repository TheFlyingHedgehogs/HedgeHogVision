from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike
import pupil_apriltags
import cv2

@dataclass
class Calibration:
    mtx: ArrayLike
    dist: ArrayLike


class PinholeCalibration(Calibration):
    pass


def perfect_camera(focal_length_mm, horizontal_sensor_size_mm, resolution) -> PinholeCalibration:
    focal_length = (resolution[0] / horizontal_sensor_size_mm) * focal_length_mm

    mtx = np.array([[focal_length,          0.0, resolution[0] / 2],
                    [0.0,          focal_length, resolution[1] / 2],
                    [0.0,                   0.0, 1.0              ]])

    dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

    return PinholeCalibration(mtx, dist)

class FoundTag:
    id: int
    translation: ArrayLike
    rotation: ArrayLike


class Detector:
    def __init__(self, calibration: Calibration, tag_width_m: float = 0.1524):
        self.calibration = calibration
        self.detector = pupil_apriltags.Detector(families="tag16h5")  # TODO test thread count

        tag_half = tag_width_m / 2
        self.object_points = np.array([
            [-tag_half, -tag_half, 0.0],
            [tag_half, -tag_half, 0.0],
            [tag_half, tag_half, 0.0],
            [-tag_half, tag_half, 0.0]
        ], dtype=np.float64)
    
    def find_tags(self, image: ArrayLike) -> list[FoundTag()]:
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        found = self.detector.detect(gray)
        # print(found)

        detected = []

        for item in found:
            if item.hamming > 0:
                continue
            image_points = np.array(item.corners)

            i = 1
            for c in item.corners:
                cv2.drawMarker(img, (int(c[0]), int(c[1])), color=(0,255 / i,0), markerType=cv2.MARKER_TILTED_CROSS, thickness=1)
                i += 1

            ret, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.calibration.mtx, self.calibration.dist)
            if ret:
                print(f"tag {item.tag_id}")
                print("rvec")
                print(rvec)

                

                print("tvec")
                print(tvec)
                print()

                cv2.drawFrameAxes(img, self.calibration.mtx, self.calibration.dist, rvec, tvec, 0.1524)
            # cv2.imshow("a", img)
            # cv2.waitKey()
            # cv2.waitKey()
            # cv2.waitKey()
            # cv2.waitKey()
            # cv2.waitKey()
            # cv2.waitKey()
            # cv2.waitKey()

# img = cv2.imread("/tmp/0250.png")
img = cv2.imread("/home/foo/synced/2023-field/angles/45.png")
# img = cv2.imread("/tmp/Untitled.png")
# img = cv2.imread("/home/foo/synced/2023-field/tags/tag16_05_00005.png")
calibration = perfect_camera(50, 36, (1920, 1080))

# d = pupil_apriltags.Detector(families="tag16h5")
# d = dt_apriltags.Detector(families='tag16h5')
# print(d.detect(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)))

detector = Detector(calibration)

detector.find_tags(img)
