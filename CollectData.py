import time

from Detector import Detector
from camera import getImage
from math_stuff.transform3d import Transform3d
from statistics import mean
from statistics import stdev
import time
import copy

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

def collectFakeSD(tag_finder: Detector) -> None:
    """Starts the main loop, and prints values for FPS """
    frames = 0

    while True:
        listofOutputs = []
        location = Transform3d.zero()
        lastimg = getImage()
        for i in range(100):
            time.sleep(0.01)
            # while location.translation.is_zero():
            newImg = getImage()
            location = tag_finder.get_world_pos_from_image(newImg)
            listofOutputs.append(location)
        def getX(a: Transform3d): return a.translation.x
        def getY(a: Transform3d): return a.translation.y
        def getZ(a: Transform3d): return a.translation.z
        print(f"-====== X: {list(map(getX, listofOutputs))}")

        meanX = mean(map(getX, listofOutputs))
        meanY = mean(map(getY, listofOutputs))
        meanZ = mean(map(getZ, listofOutputs))
        print(f"Means: [x: {meanX}, y: {meanY}, z: {meanZ}]")
        XStandardDev = stdev(map(getX, listofOutputs),meanX)
        YStandardDev = stdev(map(getY, listofOutputs),meanY)
        ZStandardDev = stdev(map(getZ, listofOutputs),meanZ)
        print(f"Standard Deviations: [x: {XStandardDev}, y: {YStandardDev}, z: {ZStandardDev}]")
        input("--+-- ==+== New Test ==+== --+--")

