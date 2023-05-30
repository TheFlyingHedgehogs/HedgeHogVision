from HedgeHogVision.Camera.Calibration import PinholeCalibration, perfect_camera
from HedgeHogVision.SmartDashboard.dashboard import VisionNetworkTable
from HedgeHogVision.Detector.AdjecencyDetector import AdjecencyDetector
from HedgeHogVision.Detector.IndividualDetector import IndividualDetector
from HedgeHogVision.math_stuff.math_stuff import Transform3d, Translation3d, Rotation3d
import pickle as pkl

from HedgeHogVision.HedgeHogDetector import HedgeHogDetector, DetectorType

import cv2
from HedgeHogVision.Tags.Tags import field

#VisionNetworkTable.fromString("10.28.98.2","Vision","SmartDashboard/Odometry")
#mtx, dist = pkl.load(open("calib-picam-4", "rb"))
#calibration = PinholeCalibration(mtx, dist)
calibration = perfect_camera(50, 36, (1920, 1080))

#cv2.QT_QPA_PLATFORM = "wayland"
#tag_finder = AdjecencyDetector(calibration)
_robotToCamera = Transform3d(Translation3d(-0.1397,0,-0.16), Rotation3d.zero())
img = cv2.imread("/home/ozy/Documents/Tag/test5_7.png")

print(field[5].pose)

detectorA = HedgeHogDetector(
    DetectorType.ADJACENCY,
    calibration,
    field,
    cameraOffset=_robotToCamera
)
detectorB = HedgeHogDetector(
    DetectorType.INDIVIVIDUAL,
    calibration,
    field,
    cameraOffset=_robotToCamera
)


HedgeHogDetector.compare_detectors(img, detectorA, detectorB, Translation3d(5, 0, 7),
                                   method1Name = "Adjacency", method2Name = "Individual")

#start(tag_finder)
#debug(tag_finder)
