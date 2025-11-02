import numpy as np
import cv2 as cv
import glob
 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
calibration_flags = cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv.fisheye.CALIB_FIX_SKEW#+cv2.fisheye.CALIB_CHECK_COND

 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
CHECKERBOARD = (6, 9)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
# Load video stream
vs = cv.VideoCapture(0)

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
its = 0
 
while active and its < 1000:
    active, img = vs.read()
    h, w, c = img.shape
    img = cv.resize(img, (int(w/2), int(h/2)))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH+cv.CALIB_CB_FAST_CHECK+cv.CALIB_CB_NORMALIZE_IMAGE)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2.copy())
 
        # Draw and display the corners
        img = cv.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    
    cv.imshow('Camera Image',img)
    k = cv.waitKey(10)
    if k & 0xFF == ord("q"):
        cv.destroyAllWindows()
        break
    its += 1

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(ret)
print(mtx)
print(dist)
print(rvecs)
print(tvecs)
# Calibrate camera
N_OK = len(objpoints)
Kfish = np.zeros((3, 3))
Dfish = np.zeros((4, 1))
rvecsfish = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecsfish = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
retfish, Kfish, Dfish, rvecsfish, tvecsfish = cv.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        Kfish,
        Dfish,
        rvecsfish,
        tvecsfish,
        calibration_flags,
        (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(img.shape[::-1]))
print(retfish)
print(Kfish)
print(Dfish)
print(rvecsfish)
print(tvecsfish)


# Find the error for recalibration
mean_error = 0
mean_error_fish = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
    imgpoints2fish, _ = cv.fisheye.projectPoints(objpoints[i], rvecsfish[i], tvecsfish[i], Kfish, Dfish)
    errorfish = cv.norm(imgpoints[i], imgpoints2fish, cv.NORM_L2)/len(imgpoints2fish)
    mean_error_fish += error_fish
 
print( "total error: {}".format(mean_error/len(objpoints)) )
print( "total fish error: {}".format(mean_error_fish/len(objpoints)) )



# Get optimal camera matrix for chessboard region
h, w, c = img.shape
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
# Undistort and crop
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv.imshow('Undistort', dst)



# Undistort
h, w, c = img.shape
newcameramtx, roi = cv.getOptimalNewCameraMatrix(Kfish, Dfish, (w,h), 1, (w,h))
mapx, mapy = cv.fisheye.initUndistortRectifyMap(Kfish, Dfish, None, newcameramtx, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imshow('Undistort_fish', dst)

 
cv.destroyAllWindows()