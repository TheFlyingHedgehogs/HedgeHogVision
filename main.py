import cv2
from Calibration import perfect_camera
from Calibration import PinholeCalibration
from Detector import Detector
from main_loop import start, debug
import pickle as pkl

# img = cv2.imread("C:\\Users\\ozypf\\Downloads\\tags\\field3.png")
# calibration = perfect_camera(50, 36, (1920, 1080))
mtx, dist = pkl.load(open("calib-picam-0", "rb"))
calibration = PinholeCalibration(mtx, dist)

detector = Detector(calibration)
# detector = Detector()
# tags = detector.find_tags(img)
# start(detector)

debug(Detector)
#print(detector.get_world_pos_from_image(img))
