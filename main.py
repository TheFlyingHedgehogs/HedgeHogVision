import cv2
from Calibration import PinholeCalibration
from Detector.IndividualDetector import IndividualDetector
from Detector.ConstructorDetector import ConstructorDetector
from main_loop import start, debug
import pickle as pkl

mtx, dist = pkl.load(open("calib-picam-4", "rb"))
calibration = PinholeCalibration(mtx, dist)
#cv2.QT_QPA_PLATFORM = "wayland"
tag_finder = ConstructorDetector(calibration)

start(tag_finder)
#debug(tag_finder)
