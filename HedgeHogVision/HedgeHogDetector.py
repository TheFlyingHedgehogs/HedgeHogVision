import HedgeHogVision.Detector.AdjecencyDetector
from HedgeHogVision.math_stuff.math_stuff import Translation3d, Transform3d
from HedgeHogVision.SmartDashboard.dashboard import VisionNetworkTable
from numpy.typing import ArrayLike
from HedgeHogVision.Tags.Tags import Field
from HedgeHogVision.Detector.AdjecencyDetector import AdjecencyDetector
from HedgeHogVision.Detector.IndividualDetector import IndividualDetector
import cv2

from enum import Enum
class DetectorType(Enum):
    ADJACENCY = AdjecencyDetector
    INDIVIVIDUAL = IndividualDetector


class HedgeHogDetector:
    def __init__(self, detector: DetectorType,
                 calibration,
                 field: Field,
                 camera = None,
                 cameraOffset: Transform3d = Transform3d.zero(),
                 networkTable: VisionNetworkTable = None,
                 solveType = cv2.SOLVEPNP_ITERATIVE):
        """
        :param detector: The detector class of the method you want to use. The recommended detector is the AdjecencyDetector (Do not instantiate the class)
        :param calibration: The calibration of the camera you are using.
        :param camera: The camera object from which to get images to proccess. Not required if you are instead passing images as an object or if you are using static images.
        :param cameraOffset The translation from the center of the robot to the camera.
        """
        self.detector = detector.value(calibration,field)
        self.camera = camera
        self.cameraOffset = cameraOffset
        self.networkTable = networkTable
        self.detector.solveType = solveType
    def solveImage(self, image: ArrayLike):
        if(self.networkTable != None): self.detector.roborioPosition = self.networkTable.getRoborioPosition()
        return self.detector.get_world_pos_from_image(image).transform_by(self.cameraOffset.inverse())
    def debug(self, image: ArrayLike):
        tags, poses, avrg = self.detector.debug_pos_from_image(image)
        for i in tags:
            i.info()

        print("\n")
        print("Real Position")
        print(avrg)
    @staticmethod
    def compare_detectors(image, detector1, detector2, translation: Translation3d, method1Name = "Method 1", method2Name = "Method 2", ignoreY: bool = True):
        """
        :param image:
        :param detector1:
        :param detector2:
        :param translation:
        :param ignoreY:
        :param method1Name:
        :param method2Name:
        :return: None
        """
        result1: Transform3d = detector1.solveImage(image)
        result2: Transform3d = detector2.solveImage(image)
        result1Distance = result1.translation.field_distance2D(translation)
        result2Distance = result2.translation.field_distance2D(translation)
        evaluation = ""
        if(result1Distance < result2Distance):
            evaluation = f"{method1Name} performed better, with an inaccuracy of {result1Distance} meters\n"
        else:
            evaluation = f"{method2Name} performed better, with an inaccuracy of {result2Distance} meters\n"
        evaluation += "Inaccuracy:\n\t"
        evaluation += f"{method1Name}: {result1Distance}\n\t"
        evaluation += f"{method2Name}: {result2Distance}\n"

        evaluation += "Results:\n"
        evaluation += f"{method1Name}:{result1}\n"
        evaluation += f"{method2Name}:{result2}\n"
        print(evaluation)

