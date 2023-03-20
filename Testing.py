from Detector.IndividualDetector import IndividualDetector
from Detector.ConstructorDetector import ConstructorDetector
from Detector.AdjecencyDetector import AdjecencyDetector

#from CollectData import collectSD, collectFakeSD
from Calibration import perfect_camera
import Tags
from Tags import KnownTag
import cv2

img = cv2.imread("/home/ozy/Documents/Tag/CoolerTest4.9_5.png")
calibration = perfect_camera(50, 36, (1920, 1080))

print(Tags.field)
individual = IndividualDetector(calibration)
constructor = AdjecencyDetector(calibration)

print("--++== INDIVIDUAL ==++--")
poses = individual.characterize_vision(img)
for i in poses:
    print(i)
print("--++== CONSTRUCTOR ==++--")
poses = constructor.characterize_vision(img)
for i in poses:
    print(i)
