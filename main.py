from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike
import pupil_apriltags
import cv2
import math
from math_stuff.math_stuff import Transform3d, Translation3d, Pose3d, Rotation3d
from Calibration import perfect_camera
from Detector import Detector
from pyquaternion import Quaternion

img = cv2.imread("C:\\Users\\ozypf\\Downloads\\tags\\fieldTest5.png")
calibration = perfect_camera(50, 36, (1920, 1080))


detector = Detector(calibration)

tags = detector.find_tags(img)
transforms = []
for i in tags:
    roboLocation = i.get_robot_location()
    transforms.append(roboLocation)
    print(f"transforms{roboLocation}")
print(f"Averaged: {Transform3d.average(transforms)}")