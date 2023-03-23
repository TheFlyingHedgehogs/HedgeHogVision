import time

from dashboard import SmartDashboard, NetworkTables
from Detector import Detector
from camera import getImage
import time

def start(tag_finder: Detector) -> None:
    """Starts the main loop, putting value to smart dashboard"""
    while True:
        tag_finder.update()

        start_time = time.time()
        pos, stdev = tag_finder.get_world_pos_with_deviation(getImage())
        #print("Position: ")
        #print(pos.translation)
        #print("STDEV")
        #print(stdev.translation)

        stdev.to_smart_dashboard("VisionStdDev")
        pos.to_smart_dashboard("VisionPos", time.time()-start_time)
        NetworkTables.flush()



def debug(tag_finder: Detector) -> None:
    """Starts the main loop, and prints values for FPS """
    frames = 0

    while True:
        tag_finder.update()
        pos, stdev = tag_finder.get_world_pos_with_deviation(getImage())
        print("Position: ")
        print(pos.translation)
        print("STDEV")
        print(stdev.translation)

        pos.to_smart_dashboard("VisionPos")
        stdev.to_smart_dashboard("VisionStdDev")
        """frames += 1
        if frames >= 100:
            now = time.perf_counter()
            print(f"avg fps: {frames / (now - start)}")
            start = now
            frames = 0
        """
        print("---+===-{ New Frame }-===+---")