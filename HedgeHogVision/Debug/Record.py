from datetime import datetime
import time
from HedgeHogVision.Detector import Detector
from HedgeHogVision.Camera.camera import getImage

def startRecoding(tag_finder: Detector) -> None:
    """Starts the main loop, and prints values for FPS """
    f = open("M","r+")
    test_num = f.readline(0)
    f.write(str(int(test_num + 1)))
    f.close()
    while True:
        time.sleep(1)
        img = getImage()
        location, leng = tag_finder.get_world_pos_from_image(img)
        now = datetime.now()
        current_time = now.strftime("%H: %M: %S")
        f = open(f"Test #{test_num}", "a")
        f.write(f'\n{current_time}' + location)
        f.close()
        location.to_smart_dashboard()
