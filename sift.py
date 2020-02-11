import numpy as np
import glob
import cv2

files = glob.glob('.\\output_images\\*.jpg')


for fname in files:
    print(fname)
    img = cv2.imread(fname)
    #gray= cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    orb_im = np.array(0)
    img.copyTo(orb_im)
    orb = cv2.ORB_create()
    kp, des = orb.detectAndCompute(img, None)
    
    cv2.drawKeypoints(img, kp, orb_im, color=(0,0,255))
    cv2.namedWindow('orb', cv2.WINDOW_NORMAL)
    cv2.imshow('orb', orb_im)





    cv2.waitKey(-1)
    #cv.imwrite('sift_keypoints.jpg',img)

cv2.destroyAllWindows()