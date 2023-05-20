from HedgeHogVision.Detector.IndividualDetector import IndividualDetector
from HedgeHogVision.Detector.AdjecencyDetector import AdjecencyDetector

#from CollectData import collectSD, collectFakeSD
from HedgeHogVision.Camera.Calibration import perfect_camera
from HedgeHogVision.Tags import Tags
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
