import HedgeHogVision.Detector.AdjecencyDetector
from HedgeHogVision.math_stuff.math_stuff import Translation3d, Transform3d
from HedgeHogVision.SmartDashboard.dashboard import VisionNetworkTable
from numpy.typing import ArrayLike
from HedgeHogVision.Tags.Tags import KnownTag

class HedgeHogDetector:
    def __init__(self, detector,
                 calibration,
                 field: list[KnownTag],
                 camera = None,
                 cameraOffset: Transform3d = Transform3d.zero(),
                 networkTable: VisionNetworkTable = None):
        """
        :param detector: The detector class of the method you want to use. The recommended detector is the AdjecencyDetector (Do not instantiate the class)
        :param calibration: The calibration of the camera you are using.
        :param camera: The camera object from which to get images to proccess. Not required if you are instead passing images as an object or if you are using static images.
        :param cameraOffset The translation from the center of the robot to the camera.
        """
        self.detector = detector(calibration,field)
        self.camera = camera
        self.cameraOffset = cameraOffset
        self.networkTable = networkTable
    def solveImage(self, image: ArrayLike):
        if(self.networkTable != None): self.detector.roborioPosition = self.networkTable.getRoborioPosition()
        return self.detector.get_world_pos_from_image(image).transform_by(self.cameraOffset.inverse())
    def debug(self, image: ArrayLike):
        return self.detector.get_world_pos_from_image(image).transform_by(self.cameraOffset.inverse())
