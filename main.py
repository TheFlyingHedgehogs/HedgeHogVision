import cv2
from Calibration import perfect_camera
from Calibration import PinholeCalibration
from Detector import Detector
from main_loop import start, debug
import pickle as pkl
from CollectData import collectSD
from CollectData import collectFakeSD

# img = cv2.imread("C:\\Users\\ozypf\\Downloads\\tags\\field3.png")
# calibration = perfect_camera(50, 36, (1920, 1080))
mtx, dist = pkl.load(open("calib-picam-0", "rb"))
calibration = PinholeCalibration(mtx, dist)

tag_finder = Detector(calibration)

#start(detector)
collectFakeSD(tag_finder)
