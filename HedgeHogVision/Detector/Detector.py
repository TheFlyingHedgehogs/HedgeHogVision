import cv2
import numpy as np
import pupil_apriltags
from numpy.typing import ArrayLike
from HedgeHogVision.Camera.Calibration import Calibration
from HedgeHogVision.Tags.Tags import FoundTag, KnownTag
from HedgeHogVision.math_stuff.math_stuff import Transform3d
from HedgeHogVision.math_stuff.rotation3d import Rotation3d
from HedgeHogVision.math_stuff.translation3d import Translation3d
from abc import ABC
from abc import abstractmethod

"""
b    g
+---+
|   |
+---+
p    r
"""

class Detector(ABC):
    """Used to find Apriltags in an image and return a position on the field (ABSTRACT CLASS, DO NOT INSTANTIATE)"""
    def __init__(self, calibration: Calibration, field: list[KnownTag]):
        self.lastKnownPosition: Transform3d = Transform3d.zero()
        self.roborioPosition: Translation3d = None
        self.time_since_last_update = 0

        self.calibration = calibration
        self.detector = pupil_apriltags.Detector(families="tag16h5", nthreads=4)  # TODO test thread count
        self.field = field

        """tag_half = tag_width_m / 2
        self.tag_half = tag_half
        self.object_points = np.array([
            [-tag_half,  tag_half, 0.0],
            [ tag_half,  tag_half, 0.0],
            [ tag_half, -tag_half, 0.0],
            [-tag_half, -tag_half, 0.0]
        ], dtype=np.float64)"""

    @abstractmethod
    def create_tags(self, tags) -> list[list[FoundTag]]: pass

    def find_tags(self, image: ArrayLike):
        """
        :rtype list[cv2.Detection]:
        :param image: Image to find tags in
        :return: A list of tag detections in the images, filtered by hamming and decision margin
        """
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        found = self.detector.detect(gray)
        firstFilter = list(filter(lambda item: not (item.hamming > 0 or item.decision_margin < 0.7 or item.tag_id >= len(self.field)), found))
        return list(filter(lambda item : self.field[item.tag_id] is not None, firstFilter))
    def __get_cluster(self, startPair: list[FoundTag,FoundTag], pairList: list[list[FoundTag,FoundTag]]) -> tuple[list[FoundTag], float]:
        first_tag_pos = startPair[0].robot_position
        first_total_error = 0
        first_tags = [startPair[0]]

        second_tag_pos = startPair[1].robot_position
        second_total_error = 0
        second_tags = [startPair[1]]
        for [a,b] in pairList:
            a_dist_first = first_tag_pos.field_distance(a.robot_position)
            b_dist_first = first_tag_pos.field_distance(b.robot_position)

            if a_dist_first < b_dist_first: first_tags.append(a)
            else: first_tags.append(b)
            error = min(a_dist_first, b_dist_first)
            first_total_error += error

            a_dist_second = second_tag_pos.field_distance(a.robot_position)
            b_dist_second = second_tag_pos.field_distance(b.robot_position)
            if a_dist_second < b_dist_second: second_tags.append(a)
            else: second_tags.append(b)
            error = min(a_dist_second, b_dist_second)
            second_total_error += error

        if first_total_error < second_total_error: return first_tags, first_total_error
        else: return second_tags, second_total_error
    def trimmed_tags(self, tags: list[list[FoundTag,FoundTag]]) -> list[FoundTag]:
        """Returns a list of tags, trimming the tag on the incorrect side from the pairs"""
        if len(tags) == 0 or tags == None or None in tags: return []
        if len(tags) == 1:
            if self.roborioPosition is not None:
                return [tags[0][self.roborioPosition.field_distance(tags[0][0].robot_position.translation) >
                                self.roborioPosition.field_distance(tags[0][1].robot_position.translation)]]
            if self.lastKnownPosition is None:
                return [tags[0][tags[0][0].robot_position.translation.y < 0]]
            return [tags[0][self.lastKnownPosition.field_distance(tags[0][0].robot_position) >
                            self.lastKnownPosition.field_distance(tags[0][1].robot_position)]]
        firstSolve, firstError   = self.__get_cluster(tags[0], tags[1:])
        secondSolve, secondError = self.__get_cluster(tags[-1],tags[:-1])

        chosen = firstSolve if firstError < secondError else secondSolve

        return chosen
        # """Poses = list(map(lambda tag : tag.robot_position, chosen))
        # avrg = Transform3d.average(Poses)
        # stdev = max(Transform3d.standardDev(avrg, Poses),0.1)
        #
        # for i in chosen: print(i.robot_position.field_distance(avrg))
        # returnValue = list(filter(lambda t : t.robot_position.field_distance(avrg) < stdev*5, chosen))
        #
        # if(len(returnValue) < len(tags)/2): return chosen
        # return returnValue"""
    def get_world_pos_from_image(self, img: ArrayLike):
        """Used to get real world position from apriltags on the feild.
        :return: The field position of the bot
        :rtype: Transform3d
        """
        tags = self.trimmed_tags(self.create_tags(self.find_tags(img)))
        if len(tags) == 0:
            return Transform3d.zero()
        transforms = list(map(lambda tag : tag.robot_position, tags))
        position = Transform3d.average(transforms)
        self.lastKnownPosition = position
        return position
    def debug_pos_from_image(self, img: ArrayLike):
        """Used to get real world position from apriltags on the feild.
        :return: The field position of the bot
        :rtype: Transform3d
        """
        tags = self.trimmed_tags(self.create_tags(self.find_tags(img)))
        if len(tags) == 0:
            return Transform3d.zero()
        transforms = list(map(lambda tag : tag.robot_position, tags))
        position = Transform3d.average(transforms)
        self.lastKnownPosition = position
        return tags, transforms, position
    def get_world_pos_with_deviation(self, img: ArrayLike):
        """Used to get real world position from apriltags on the feild.
        :return: The field position of the bot
        :rtype: Transform3d
        """
        tags = self.find_tags(img)
        trimmedTags = self.trimmed_tags(self.create_tags(tags))
#        for i in trimmedTags:
#            print(i.robot_position)
        if len(trimmedTags) == 0: return Transform3d.zero(), Transform3d(Translation3d(999, 999, 999), Rotation3d.zero())

        transforms = list(map(lambda tag : tag.robot_position, trimmedTags))
        position = Transform3d.average(transforms)
        self.lastKnownPosition = position
        if(len(tags) == 1):
            dev = max(2.0, Transform3d.average(list(map(lambda tag : tag.tag_transform, trimmedTags))).translation.z)/2
            stdev = Transform3d(Translation3d(dev, dev, dev),Rotation3d.zero())
        else:
            stdev = (Transform3d.averageDistanceTo(transforms, position) * 2)/len(tags)

        return position, stdev