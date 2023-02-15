import cv2
import numpy
import numpy as np
import pupil_apriltags
from numpy.typing import ArrayLike
from Calibration import Calibration
from Tags import FoundTag, field, MegaTag
from math_stuff.math_stuff import Transform3d
from dashboard import SmartDashboard
from math_stuff.rotation3d import Rotation3d
from math_stuff.translation3d import Translation3d
import time
"""
b    g
+---+
|   |
+---+
p    r
"""

class Point:
    def __init__(self,x: float = 0,y: float = 0, z: float = 0):
        self.x = x
        self.y = y
        self.z = z
        self.arr = (x, y, z)
        self.arr2d = (x, y)
    def draw(self, image, color):
        cv2.drawMarker(image, ((int(self.x), int(self.y) )), (255, 0, 0))
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __truediv__(self, other):
        if type(other) == float or type(other) == int:
            return Point(self.x/other, self.y/other)
        if type(other) == Point:
            return Point(self.x/other.x, self.y/other.y)

    def __mul__(self, other):
        if(type(other) == float or type(other) == int):
            return Point(self.x/other, self.y/other)
        if(type(other) == Point):
            return Point(self.x*other.x,self.y*other.y)
class Line:
    def __init__(self,top: Point, bot: Point):
        self.top = top
        self.bot = bot
        self.arr = (self.top.arr, self.bot.arr)
    def draw(self, image, color):
        cv2.line(image, (int(self.top.x),int(self.top.y)), (int(self.bot.x), int(self.bot.y)), color)

class Info:
    def __init__(self, real_line: Line, image_line: Line, parent: MegaTag):
        self.real_line = real_line
        self.image_line = image_line
        self.parent = parent
class Detector:
    """Used to find Apriltags in an image and return a position on the field"""
    def update(self):
        self.roborioPosition = Translation3d(Translation3d.getValue("RobotX"), 0, Translation3d.getValue("RobotY"))
    def __init__(self, calibration: Calibration, tag_width_m: float = 0.1524):
        self.lastKnownPosition: Transform3d = None
        self.roborioPosition: Translation3d = None
        self.listOfLastPos = []
        self.time_since_last_update = 0

        self.calibration = calibration
        self.detector = pupil_apriltags.Detector(families="tag16h5", nthreads=4)  # TODO test thread count

        tag_half = tag_width_m / 2
        self.tag_half = tag_half
        self.object_points = np.array([
            [-tag_half,  tag_half, 0.0],
            [ tag_half,  tag_half, 0.0],
            [ tag_half, -tag_half, 0.0],
            [-tag_half, -tag_half, 0.0]
        ], dtype=np.float64)

    def find_tags(self, image: ArrayLike):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        found = self.detector.detect(gray)

        return list(filter(lambda item : item.hamming > 0 or item.decision_margin < 0.7 or item.tag_id >= len(field), found)) #or item.tag_id == 0: continue

    def create_mega_tags(self, tags):

        lines = []

        for item in tags:

            pts = np.array(item.corners)
            image_points = item.corners
            """
            expected

            1   2
            +---+
            |   |
            +---+
            4   3

            4   3
            +---+
            |   |
            +---+
            1   2
            """

            c = image_points

            #cv2.drawMarker(image, ((int(c[0][0]), int(c[0][1]))), (255, 0, 0))
            #cv2.drawMarker(image, (int(c[1][0]), int(c[1][1])), (0, 255, 0))
            #cv2.drawMarker(image, (int(c[2][0]), int(c[2][1])), (0, 0, 255))
            #cv2.drawMarker(image, ((int(c[3][0]), int(c[3][1]))), (255, 0, 255))

            tag = field[item.tag_id]
            realworld_points = []
            realtopleft = Point(self.object_points[0][0] + tag.x, self.object_points[0][1] + tag.y)
            realbottomleft = Point(self.object_points[3][0] + tag.x, self.object_points[3][1] + tag.y)

            realtopright = Point(self.object_points[1][0] + tag.x, self.object_points[1][1] + tag.y)
            realbottomright = Point(self.object_points[2][0] + tag.x, self.object_points[2][1] + tag.y)

            imagetopleft = Point(image_points[0][0], image_points[0][1])
            imagebottomleft = Point(image_points[3][0], image_points[3][1])

            imagetopright = Point(image_points[1][0], image_points[1][1])
            imagebottomright = Point(image_points[2][0], image_points[2][1])

            lines.append(
                Info(
                    Line(realtopleft, realbottomleft),
                    Line(imagetopleft, imagebottomleft),
                    tag
            ))
            lines.append(
                Info(
                    Line(realtopright, realbottomright),
                    Line(imagetopright, imagebottomright),
                    tag
                )
            )
        detected = []
        lines = sorted(lines, key=lambda l : l.real_line.top.x)
        colors = [(255, 0, 0), (0, 255, 0), (0,0,255),(255,255,0),(255,0,255),(0,255,255),(255,255,255)]
        e = 0
        for i in range(len(lines)-1):
            for k in range(len(lines) - (i+1)):
                j = i + k + 1
                #print(lines[i].image_line.arr)
                #print(lines[j].image_line.arr)
                #lines[i].image_line.draw(image, (255,0,255))
                #lines[j].image_line.draw(image, (255,0,255))
                e += 1
                if(e >= len(colors)): e = 0
                # Lines[line#][realworld/image/tag][up/down][x/y]
                height = lines[i].real_line.top.y - lines[i].real_line.bot.y
                width = lines[j].real_line.bot.x - lines[i].real_line.bot.x
                widthi = lines[j].image_line.bot.x - lines[i].image_line.bot.x
                if(width == 0): continue;
                if(lines[i].parent.z != lines[j].parent.z): continue
                success, rotation_vectors, translation_vectors, _ = cv2.solvePnPGeneric(
                    np.array([
                        [-width/2, height/2, 0.0],
                        [width/2, height/2, 0.0],
                        [width/2, -height/2, 0.0],
                        [-width/2, -height/2, 0.0],
                    ], dtype=np.float64),
                    np.array([
                        lines[i].image_line.top.arr2d,
                        lines[j].image_line.top.arr2d,
                        lines[j].image_line.bot.arr2d,
                        lines[i].image_line.bot.arr2d
                    ],
                        dtype=np.float64),
                    self.calibration.mtx,
                    self.calibration.dist,
                    flags=cv2.SOLVEPNP_IPPE)
                if not success: continue
                tag_x = (lines[i].real_line.top.x + lines[j].real_line.top.x)/2
                tag_y = (lines[i].real_line.top.y + lines[i].real_line.bot.y)/2
                Line(
                    Point(lines[i].image_line.top.x, lines[i].image_line.top.y + (e*10)),
                    Point(lines[i].image_line.top.x + widthi, lines[i].image_line.top.y + (e*10))
                ).draw(image, (255,255,0))
                Point((lines[i].image_line.top.x + lines[j].image_line.top.x)/2, (lines[i].image_line.top.y + lines[i].image_line.bot.y)/2).draw(image, (0, 0, 125))
                known_tag = MegaTag(tag_x, tag_y, lines[i].parent.z, 180)
                rotation_vector, translation_vector = None, None

                to_append = []
                for (r, t) in zip(rotation_vectors,translation_vectors):
                    rotation_matrix = cv2.Rodrigues(r)[0]
                    found_tag = FoundTag(known_tag, t, rotation_matrix,item.tag_id)
                    to_append.append(found_tag)
                detected.append(to_append)
        #cv2.imshow("test",image)
        #cv2.waitKey(1000000)
        return detected

    def trimmed_tags(self, tags: list[list[FoundTag]]) -> list[FoundTag]:
        """Returns a list of tags, trimming the incorrect tags from the pairs"""
        if len(tags) == 0: return []
        """for i in tags:
            print(i[0].id)
            print(i[0].robot_position)
            print(i[1].robot_position)
            print("Tag Transform: ")
            print(i[0].tag_transform)
            print(i[1].tag_transform)
            print("First Guess")
            print(i[0].first_guess)
            print(i[1].first_guess)"""
        if len(tags) == 1:
            if(self.roborioPosition != None):
                return [tags[0][self.roborioPosition.field_distance(tags[0][0].robot_position.translation)]]
            if(self.lastKnownPosition == None): return [tags[0][tags[0][0].robot_position.translation.y < 0]]
            return [tags[0][self.lastKnownPosition.field_distance(tags[0][0].robot_position) > self.lastKnownPosition.field_distance(tags[0][1].robot_position)]]
        first_tag_pos = tags[0][0].robot_position
        first_total_error = 0
        first_tags = [tags[0][0]]

        second_tag_pos = tags[0][1].robot_position
        second_total_error = 0
        second_tags = [tags[0][1]]
        for [a,b] in tags[1:]:
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

        #print(f"Errors: {first_total_error}, {second_total_error}")
        if first_total_error < second_total_error:
            closest = first_tags
        else: closest = second_tags
        #poses = list(map(lambda t : t.robot_position, closest))
        #center = Transform3d.average(poses)
        #deviation = Transform3d.standardDev(center, poses)
        #closest = list(filter(lambda t : abs(t.robot_position.field_distance(center)) < deviation*2, closest))
        return closest

    def get_world_pos_from_image(self, img: ArrayLike):
        """Used to get real world position from apriltags on the feild.
        :return: The field position of the bot
        :rtype: Transform3d
        """
        tags = self.trimmed_tags(self.create_mega_tags(self.find_tags(img)))
        if len(tags) == 0:
            return Transform3d.zero()
        transforms = list(map(lambda tag : tag.robot_position, tags))
        position = Transform3d.average(transforms)
        self.lastKnownPosition = position
        self.listOfLastPos.append(position)
        return position

    def get_world_pos_from_image_debug(self, img: ArrayLike):
        """Used to get real world position from apriltags on the feild.
        :return: The field position of the bot
        :rtype: Transform3d
        """
        tags = self.trimmed_tags(self.create_mega_tags(self.find_tags(img)))
        for i in tags:
            print(i.robot_position)

        if len(tags) == 0:
            return Transform3d.zero(), 0
        transforms = []
        for i in tags:
            transforms.append(i.robot_position)
        position = Transform3d.average(transforms)
        self.lastKnownPosition = position
        #self.listOfLastPos.append(position)
        return position, len(tags)

    def get_world_pos_from_image_characterize(self, img: ArrayLike):
        """

        :param img:
        :return:
        """
        tags = self.find_tags(img)
        if(len(tags) < 3): print("Can't see all tags"); return None;

        tags = sorted(tags, key = lambda item : item.tag_id)
        positions = []
        for i in range(3):
            used_tags = self.trimmed_tags(self.create_mega_tags(tags[:i+1]))
            transforms = list(map(lambda tag : tag.robot_position, used_tags))
            position = Transform3d.average(transforms)
            self.lastKnownPosition = position
            positions.append(position)
        return positions




