import cv2
from Tags import MegaTag

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

    @staticmethod
    def side_points(side):
        if side == "LEFT":
            return (0, 3)
        #side = ="RIGHT"
        return (1, 2)

    @staticmethod
    def create_real_from_tag(side, object_points, tag):
        points = Line.side_points(side)
        realTop = Point(object_points[points[0]][0] + tag.x, object_points[points[0]][1] + tag.y)
        realBottom = Point(object_points[points[1]][0] + tag.x, object_points[points[1]][1] + tag.y)
        return Line(realTop, realBottom)

    @staticmethod
    def create_fake_from_image(side, image_points):
        points = Line.side_points(side)
        imageTop = Point(image_points[points[0]][0], image_points[points[0]][1])
        imageBottom = Point(image_points[points[1]][0], image_points[points[1]][1])
        return Line(imageTop, imageBottom)

class LinePair:
    def __init__(self, real_line: Line, image_line: Line, parent: MegaTag):
        self.real_line = real_line
        self.image_line = image_line
        self.parent = parent

    @staticmethod
    def create_from_info(side, object_points, image_points, tag):
        return LinePair(
            real_line=Line.create_real_from_tag(side, object_points, tag),
            image_line=Line.create_fake_from_image(side, image_points),
            parent=tag
        )