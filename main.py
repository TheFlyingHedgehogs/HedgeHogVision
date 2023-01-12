import cv2
from math_stuff import Transform3d
from Calibration import perfect_camera
from Detector import Detector

img = cv2.imread("C:\\Users\\ozypf\\Downloads\\tags\\fieldTest5.png")
# img = cv2.imread("/home/foo/synced/2023-field/angles/moved5.png")
calibration = perfect_camera(50, 36, (1920, 1080))


detector = Detector(calibration)

tags = detector.find_tags(img)
transforms = []
for i in tags:
    transforms.append(i.get_robot_location())

print(f"Averaged: {Transform3d.average(transforms)}")
