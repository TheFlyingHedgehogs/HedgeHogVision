import dataclasses
import math
import cv2
import pupil_apriltags as apriltag
import numpy as np
import pickle as pkl
from picamera2 import Picamera2
import time
from libcamera import controls
from Calibration import Calibration

@dataclasses.dataclass
class PinholeCalibration(Calibration):
    pass


@dataclasses.dataclass
class FisheyeCalibration(Calibration):
    pass

# def gen_csv(files: list[str], filename_distance_converter):
#     for i, path in enumerate(sorted(files)):
#         dst = filename_distance_converter(path)
#         gotten = detect(cv2.imread(path))[0][1]
#         found = math.sqrt(gotten[0]**2 + gotten[1]**2 + gotten[2]**2)
#         print(f"{dst},{found}")

# mtx, dist = pkl.load(open("calib-picam-0", "rb"))
# focal_length = (1920 / 36) * 100

# mtx = np.array([[focal_length,          0.0, 960],
#                 [0.0,          focal_length, 540],
#                 [0.0,                   0.0, 1.0]])
# dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

picam2 = Picamera2()
video_config = picam2.create_video_configuration({"size": (1296, 972)})
picam2.set_controls({"AeExposureMode": controls.AeExposureModeEnum.Short})
picam2.set_controls({"AnalogueGain": 100})
picam2.configure(video_config)
picam2.start()

time.sleep(2)

start = time.perf_counter()
frames = 0


def getImage():
    return picam2.capture_array("main")

