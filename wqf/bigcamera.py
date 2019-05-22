# -- coding: utf-8 --
import numpy as np
import cv2
import glob
a=0
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.png')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)


cap = cv2.VideoCapture(0)

while(1):
    try:
        ret,frame = cap.read()
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 摄像头畸变矫正
        h, w = img_gray.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
        dst = cv2.remap(img_gray, mapx, mapy, cv2.INTER_LINEAR)
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        # dst为畸变矫正后的图像
        cv2.imshow('dst1',dst)
        cany = cv2.GaussianBlur(dst, (3, 3), 0)
        edges = cv2.Canny(cany, 50, 150, apertureSize=3)
        cv2.imshow('Canny', edges)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 40)
        result = dst.copy()
        for line in lines[0]:
            rho = line[0]
            theta = line[1]
            print ('偏差',rho,'  ','角度',theta)
            if (theta < (np.pi / 4.)) or (theta > (3. * np.pi / 4.0)):

                pt1 = (int(rho / np.cos(theta)), 0)

                pt2 = (int((rho - result.shape[0] * np.sin(theta)) / np.cos(theta)), result.shape[0])

                cv2.line(result, pt1, pt2, (255))
            else:

                pt1 = (0, int(rho / np.sin(theta)))

                pt2 = (result.shape[1], int((rho - result.shape[1] * np.cos(theta)) / np.sin(theta)))

                cv2.line(result, pt1, pt2, (255), 1)
        cv2.imshow('result', result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        a=a+1
        # print(a)
        continue
cap.release()
cv2.destroyAllWindows()
