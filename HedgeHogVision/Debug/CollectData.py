from HedgeHogVision.Detector import Detector
from HedgeHogVision.Camera.camera import getImage
from HedgeHogVision.math_stuff.transform3d import Transform3d
from statistics import mean
from statistics import stdev
import time


def collectSD(tag_finder: Detector) -> None:
    """Starts the main loop, and prints values for FPS """
    frames = 0

    while True:
        ZDist = float(input("Dist from tag (Z): "))
        XDist = float(input("Dist from tag (X): "))
        listofOutputs = []
        location = Transform3d.zero()
        for i in range(100):
            while location.translation.is_zero():
                location = tag_finder.get_world_pos_from_image(getImage())
            listofOutputs.append(location)
        def getX(a: Transform3d): return abs(a.translation.x)
        def getY(a: Transform3d): return abs(a.translation.y)
        def getZ(a: Transform3d): return abs(a.translation.z)


        meanX = mean(map(getX, listofOutputs))
        meanY = mean(map(getY, listofOutputs))
        meanZ = mean(map(getZ, listofOutputs))
        print(f"Means: [x: {meanX}, y: {meanY}, z: {meanZ}]")
        XStandardDev = stdev(map(getX, listofOutputs),meanX)
        YStandardDev = stdev(map(getY, listofOutputs),meanY)
        ZStandardDev = stdev(map(getZ, listofOutputs),meanZ)
        print(f"Standard Deviations: [x: {XStandardDev}, y: {YStandardDev}, z: {ZStandardDev}]")
        realZStandardDev = stdev(map(getZ, listofOutputs),ZDist)
        realXStandardDev = stdev(map(getX, listofOutputs),XDist)
        print(f"Standard Deviations from real value: [Z: {realZStandardDev}, X: {realXStandardDev}]")
        print(f"Difference from real to estimate [z:{abs(ZDist-meanZ)}, x:{abs(XDist-meanX)}]")
        print("--+-- ==+== New Test ==+== --+--")
class TagDeviationPairs:
    def __init__(self, num_of_tags):
        self.Xpairs = []
        self.Ypairs = []
        self.Zpairs = []
        self.tags = num_of_tags
    def add(self, x_pair, y_pair, z_pair):
        self.Xpairs.append(x_pair)
        self.Ypairs.append(y_pair)
        self.Zpairs.append(z_pair)
    def __str__(self):
        ret = f"{self.tags} TAGS !!!!!!!!!!!!!!!!!----- :\n"
        ret += f"\t +-+-+-+ : X: {self.Xpairs}\n"
        ret += f"\t +-+-+-+ : Y: {self.Ypairs}\n"
        ret += f"\t +-+-+-+ : Z: {self.Zpairs}\n"
        return ret


def collectFakeSD(tag_finder: Detector) -> None:
    """Starts the main loop, and prints values for FPS """
    frames = 0
    tag_infos = (
        TagDeviationPairs(1),
        TagDeviationPairs(2),
        TagDeviationPairs(3)
    )


    while True:
        listofOutputs = []
        start = time.time()
        while len(listofOutputs) < 30:
            # while location.translation.is_zero():
            print(f"{round(len(listofOutputs)/0.3)}% done")
            newImg = getImage()
            tagCenters, tagLocs = tag_finder.get_world_pos_from_image_characterize(newImg)
            if(tagLocs != None): listofOutputs.append([tagCenters, tagLocs])

        getX = lambda transf : transf.translation.x
        getY = lambda transf : transf.translation.y
        getZ = lambda transf : transf.translation.z

        poses3 = list(map(lambda out : out[1][2], listofOutputs))
        center = listofOutputs[0][0][2]
        meanXA = mean(map(getX, poses3))
        meanYA = mean(map(getY, poses3))
        meanZA = mean(map(getZ, poses3))
        dist = center.field_distance(Transform3d.average(poses3))


        for i in range(3):
            center = listofOutputs[0][0][i]
            print(len(listofOutputs))
            print(len(listofOutputs[0]))
            poses = list(map(lambda out : out[1][i], listofOutputs))
            print(len(poses))
            print(f"Took: {(start - time.time())/100} seconds")
            #print(f"-====== X: {list(map(getX, listofOutputs))}")
            meanX = mean(map(getX, poses))
            meanY = mean(map(getY, poses))
            meanZ = mean(map(getZ, poses))
            print(f"Means: [x: {meanX}, y: {meanY}, z: {meanZ}]")
            XStandardDev = stdev(map(getX, poses),meanXA)
            YStandardDev = stdev(map(getY, poses),meanYA)
            ZStandardDev = stdev(map(getZ, poses),meanZA)
            print(f"Standard Deviations: [x: {XStandardDev}, y: {YStandardDev}, z: {ZStandardDev}]")
            tag_infos[i].add(
                (dist, XStandardDev),
                (dist, YStandardDev),
                (dist, ZStandardDev)
            )


        stop = input("--+-- ==+== New Test ==+== --+--")
        if(stop == "stop"): break
    for i in tag_infos:
        print(i)



