import time

from Detector import Detector
from camera import getImage
from math_stuff.transform3d import Transform3d
from statistics import mean
from statistics import stdev


def collectSD(tag_finder: Detector) -> None:
    """Starts the main loop, and prints values for FPS """
    frames = 0

    while True:
        XDist = float(input("Dist from tag (X): "))
        YDist = float(input("Dist from tag (Y): "))
        listofOutputs = []
        for i in range(100):
            location = tag_finder.get_world_pos_from_image(getImage())
            listofOutputs.append(location)
        def getX(a: Transform3d): return a.translation.x
        def getY(a: Transform3d): return a.translation.y
        def getZ(a: Transform3d): return a.translation.z


        meanX = mean(map(getX, listofOutputs))
        meanY = mean(map(getY, listofOutputs))
        meanZ = mean(map(getZ, listofOutputs))
        print(f"Means: [x: {meanX}, y: {meanY}, z: {meanZ}]")
        XStandardDev = stdev(map(getX, listofOutputs),meanX)
        YStandardDev = stdev(map(getX, listofOutputs),meanY)
        ZStandardDev = stdev(map(getX, listofOutputs),meanZ)
        print(f"Standard Deviations: [x: {XStandardDev}, y: {YStandardDev}, z: {ZStandardDev}]")
        realXStandardDev = stdev(map(getX, listofOutputs),XDist)
        realYStandardDev = stdev(map(getX, listofOutputs),YDist)
        print(f"Standard Deviations from real value: [x: {realXStandardDev}, y: {realYStandardDev}]")
        print(f"Difference from real to estimate [x:{abs(XDist-meanX)}, y:{abs(YDist-meanY)}]")
        print("--+-- ==+== New Test ==+== --+--")


