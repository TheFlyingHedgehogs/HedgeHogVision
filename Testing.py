from Detector.IndividualDetector import IndividualDetector
from Detector.ConstructorDetector import ConstructorDetector
#from CollectData import collectSD, collectFakeSD
from Calibration import perfect_camera
import cv2

img = cv2.imread("/home/ozy/Documents/Tag/CoolerTest1_0.png")
calibration = perfect_camera(50, 36, (1920, 1080))

individual = IndividualDetector(calibration)
constructor = ConstructorDetector(calibration)

print("--++== INDIVIDUAL ==++--")
poses = individual.characterize_vision(img)
for i in poses:
    print(i)
print("--++== CONSTRUCTOR ==++--")
poses = constructor.characterize_vision(img)
for i in poses:
    print(i)
