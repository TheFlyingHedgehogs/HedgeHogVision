from HedgeHogVision.Camera.Calibration import PinholeCalibration, perfect_camera
from HedgeHogVision.SmartDashboard.dashboard import VisionNetworkTable
from HedgeHogVision.Detector.AdjecencyDetector import AdjecencyDetector
from HedgeHogVision.Detector.IndividualDetector import IndividualDetector
from HedgeHogVision.math_stuff.math_stuff import Transform3d, Translation3d, Rotation3d
import pickle as pkl

from HedgeHogVision.Tags.Tags import KnownTag, Field, ChargedUp2023

from HedgeHogVision.HedgeHogDetector import HedgeHogDetector, DetectorType

import cv2

#VisionNetworkTable.fromString("10.28.98.2","Vision","SmartDashboard/Odometry")
#mtx, dist = pkl.load(open("calib-picam-4", "rb"))
#calibration = PinholeCalibration(mtx, dist)
calibration = perfect_camera(50, 36, (1920, 1080))

#cv2.QT_QPA_PLATFORM = "wayland"
#tag_finder = AdjecencyDetector(calibration)
_robotToCamera = Transform3d(Translation3d(-0.1397,0,-0.16), Rotation3d.zero())
def getImage(fileName):
    return cv2.imread(f"/home/ozy/Documents/Tag/{fileName}.png")

img = getImage("test5_7")

print(ChargedUp2023[5].pose)

field = Field(
    KnownTag(1, 0, 0, 10, 0),
    KnownTag(2, 2, 0, 10, 0)
)

detectorA = HedgeHogDetector(
    DetectorType.ADJACENCY,
    calibration,
    ChargedUp2023,
    cameraOffset=_robotToCamera
)
detectorB = HedgeHogDetector(
    DetectorType.INDIVIVIDUAL,
    calibration,
    field,
    cameraOffset=_robotToCamera,
    solveType=cv2.SOLVEPNP_IPPE
)

HedgeHogDetector.compare_detectors(img, detectorA, detectorB, Translation3d(5, 0, 7),
                                   method1Name = "Iterative", method2Name = "IPPE")

#start(tag_finder)
#debug(tag_finder)
