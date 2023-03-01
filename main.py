import cv2
from Calibration import perfect_camera
from Calibration import PinholeCalibration
from MegaTagDetector import Detector as MegaDetector
from Detector import Detector as Detector
from main_loop import start, debug
import pickle as pkl
#from CollectData import collectSD
#from CollectData import collectFakeSD
from Record import startRecoding
#img = cv2.imread("/home/ozy/Documents/Tag/Test4.png")
#calibration = perfect_camera(50, 36, (1920, 1080))
mtx, dist = pkl.load(open("calib-picam-0", "rb"))
calibration = PinholeCalibration(mtx, dist)
cv2.QT_QPA_PLATFORM = "wayland"
tag_finder = Detector(calibration)
mega_tag_finder = MegaDetector(calibration)

#print(tag_finder.get_world_pos_from_image_normal(img)[0])
#pos, le = tag_finder.get_world_pos_from_image_debug(img)
#pos2, le2 = mega_tag_finder.get_world_pos_from_image_debug(img)
#print(pos)
#print(le)

#start(detector)
#debug(mega_tag_finder)
debug(mega_tag_finder)
