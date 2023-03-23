import cv2
from Calibration import PinholeCalibration
from Detector.IndividualDetector import IndividualDetector
from Detector.ConstructorDetector import ConstructorDetector
from Detector.AdjecencyDetector import AdjecencyDetector
from main_loop import start, debug
import pickle as pkl

mtx, dist = pkl.load(open("calib-picam-4", "rb"))
calibration = PinholeCalibration(mtx, dist)
#cv2.QT_QPA_PLATFORM = "wayland"
tag_finder = AdjecencyDetector(calibration)

start(tag_finder)
#debug(tag_finder)
