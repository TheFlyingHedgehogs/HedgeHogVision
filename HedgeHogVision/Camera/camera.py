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
class VisionPiCamera:
    def __init__(self, size = (1296, 972)):
        self.camera = Picamera2()
        video_config = self.camera.create_video_configuration({"size": size})
        self.camera.set_controls({"AeExposureMode": controls.AeExposureModeEnum.Short})
        self.camera.set_controls({"AnalogueGain": 100})
        self.camera.configure(video_config)
        self.camera.start()

        time.sleep(2)

        start = time.perf_counter()
        frames = 0
        def getImage(self):
            return self.camera.capture_array("main")

