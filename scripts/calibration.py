import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 0, 0.0001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)

hor = 10
ver = 15

objp = np.zeros((hor*ver,3), np.float32)
objp[:,:2] = 14.2*np.mgrid[0:hor,0:ver].T.reshape(-1,2)


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('camera_cal/*.jpg')


sz = [0,0]

for fname in images:
    img = cv2.imread(fname)
    sz = img.shape[:2]

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (hor,ver), cv2.CALIB_CB_ADAPTIVE_THRESH)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (hor,ver), corners2, ret)
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        cv2.imshow('img', img)
        cv2.waitKey(50)



    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, sz, None, None)

    out = open('.\\output_images\\camera_calib.txt','w')
    out.write('M = \n')
    out.write(np.array_str(mtx))
    out.write('\nD = \n')
    out.write(np.array_str(dist))
    out.close()

cv2.destroyAllWindows()

for fname in images:
    print(fname)
    img = cv2.imread(fname)
    
    # undistort
    dst = cv2.undistort(img, mtx, dist, None, mtx)

    cv2.namedWindow('rect', cv2.WINDOW_NORMAL)
    cv2.imshow('rect', dst)
    cv2.waitKey(50)
    newName = '.\\output_images\\undistorted_' + fname.split('\\')[-1]
    cv2.imwrite(newName, dst)
    #cv2.imwrite('calibresult.png',dst)

cv2.destroyAllWindows()

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
    
print("total error: ", mean_error/len(objpoints))