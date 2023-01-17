import os
from multiprocessing import Pool

import cv2
import numpy as np
import glob
import pickle as pkl

# Defining the dimensions of checkerboard
from calib_thread import frame

CHECKERBOARD = (6, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []


# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
# s = (12.9 / 6) / 1000
#s = (139.38 / 5) / 1000
for col in objp:
    for row in col:
        row[0] *= 27.05 / 1000
        row[1] *= 27.53 / 1000
        # row[0] *= s
        # row[1] *= s
        # row[2] *= s

prev_img_shape = None

# Extracting path of individual image stored in a given directory
images = list(glob.glob("images/picam-0/*"))


if __name__ == '__main__':
    with Pool(processes=16) as p:
        mapped = list(p.map(frame, images))
        for item in mapped:
            if item is not None:
                # ob, im = item
                imgpoints.append(item)
                objpoints.append(objp)

    # for fname in images:
    #     img = cv2.imread(fname)
    #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #     # Find the chess board corners
    #     # If desired number of corners are found in the image then ret = true
    #     ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    #
    #     """
    #     If desired number of corner are detected,
    #     we refine the pixel coordinates and display
    #     them on the images of checker board
    #     """
    #     if ret:
    #         print(".", end="")
    #         objpoints.append(objp)
    #         # refining pixel coordinates for given 2d points.
    #         corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    #
    #         imgpoints.append(corners2)
    #
    #         # # Draw and display the corners
    #         # img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    #     else:
    #         print("x", end="")
    #
    #     # cv2.imshow('img', img)
    #     # cv2.waitKey(0)

    # cv2.destroyAllWindows()

    h, w = 972, 1296
    # h, w = img.shape[:2]

    """
    Performing camera calibration by 
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the 
    detected corners (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)

    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)

    if os.path.exists("calib-picam-0"):
        os.remove("calib-picam-0")

    with open("calib-picam-0", "xb") as f:
        pkl.dump((mtx, dist), f)

    # print("rvecs : \n")
    # print(rvecs)
    # print("tvecs : \n")
    # print(tvecs)
