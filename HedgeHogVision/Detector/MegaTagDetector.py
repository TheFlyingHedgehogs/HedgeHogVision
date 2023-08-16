import cv2
import numpy as np
from ..Camera.Calibration import Calibration
from ..Tags.Tags import FoundTag, MegaTag
from multiprocessing import Pool
from .Detector import Detector
from .Utils import LinePair
import math

class AdjecencyDetector(Detector):
    """Used to find Apriltags in an image and return a position on the field
    MegaTagDetector: Uses corners of tags to create new tags, and find positions of those fake tags"""
    def create_tags(self, tags) -> list[list[FoundTag]]:
        """
        Finds edges of the tag, finds out their real-world position.
        Then it reconstructs the tags, and uses SolvePNP to find the real-world distance from the camera to the tags.
        Because there is ambiguity across the x axis on SolvePNP's position, this function returns a list pairs of tags, one of them correct, and another one on the wrong side of the tag
        To throw out the incorrect tags, use a function such as "trimmed_tags"
        :param tags: List of Detections to find the corners of
        :return: List of tag pairs
        """
        if len(tags) == 0: return []

        lines = []

        for item in tags:
            image_points = item.corners

            tag = self.field[item.tag_id]

            lines.append(LinePair.create_from_info("LEFT", tag.object_points, image_points, tag))
            lines.append(LinePair.create_from_info("RIGHT", tag.object_points, image_points, tag))

        line1 = arg[0]
        line2 = arg[1]
        calibration = arg[2]
        height = line1.real_line.top.y - line1.real_line.bot.y
        width = line2.real_line.bot.x - line1.real_line.bot.x
        #print(line1.real_line.bot.x)
        if line1.parent.z != line2.parent.z: return
        rotation = line1.parent.rotation
        z1 = 0.0
        z2 = 0.0
        objectPoints = np.array([
            [-width/2, height/2, z1],
            [width/2, height/2, z2],
            [width/2, -height/2, z2],
            [-width/2, -height/2, z1],
        ], dtype=np.float64)
        imagePoints = np.array([
            line1.image_line.top.arr2d,
            line2.image_line.top.arr2d,
            line2.image_line.bot.arr2d,
            line1.image_line.bot.arr2d
        ],
            dtype=np.float64)
        success, rotation_vectors, translation_vectors, _ = cv2.solvePnPGeneric(
            objectPoints,
            imagePoints,
            calibration.mtx,
            calibration.dist,
            flags=self.solveType)
        if not success: return
        tag_x = (line2.real_line.top.x + line1.real_line.top.x)/2
        tag_y = (line1.real_line.top.y + line1.real_line.bot.y)/2
        known_tag = MegaTag(tag_x, tag_y, (line1.parent.z + line2.parent.z)/2, math.degrees(rotation), width)
        print(imagePoints)
        tagWidth = line2.image_line.bot.x - line1.image_line.bot.x
        tagHeight = line1.image_line.top.y - line1.image_line.bot.y
        print(f"T1: {line1.parent.id}, T2: {line2.parent.id}: TW: {tagWidth}, TH: {tagHeight}")
        to_append = []
        for (r, t) in zip(rotation_vectors,translation_vectors):
            rotation_matrix = cv2.Rodrigues(r)[0]
            found_tag = FoundTag(known_tag, t, rotation_matrix, f"T1: {line1.parent.id}, T2: {line2.parent.id}")
            to_append.append(found_tag)
        return to_append
        args = []
        for i in range(len(lines)-1):
            j = i + 1
            args.append([lines[i], lines[j], self.calibration, i, j, self.solveType])
        pool = Pool(4)

        detected = list(filter(lambda a : a != None, pool.map(process_tag, args)))

        if detected is None: return []
        return detected