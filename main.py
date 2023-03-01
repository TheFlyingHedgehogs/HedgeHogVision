import cv2
from Calibration import perfect_camera
from Calibration import PinholeCalibration
from Detector.ConstructorDetector import ConstructorDetector
from main_loop import start, debug
import pickle as pkl
#from CollectData import collectSD, collectFakeSD
#img = cv2.imread("/home/ozy/Documents/Tag/Test4.png")
#calibration = perfect_camera(50, 36, (1920, 1080))
mtx, dist = pkl.load(open("calib-picam-0", "rb"))
calibration = PinholeCalibration(mtx, dist)
#cv2.QT_QPA_PLATFORM = "wayland"
tag_finder = ConstructorDetector(calibration)

#start(detector)
debug(tag_finder)
