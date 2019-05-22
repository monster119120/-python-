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
images = glob.glob('piccam\\*.png')
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





def nothing(x):
    pass
cv2.namedWindow('dst1')
cv2.createTrackbar('minLineLength','dst1',20,100,nothing)
cv2.createTrackbar('maxLineGap','dst1',20,100,nothing)
cv2.createTrackbar('threshod','dst1',150,500,nothing)



cap = cv2.VideoCapture(1)
black = np.zeros((300,512,3),np.uint8)
while(1):
    minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
    maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
    threshod = cv2.getTrackbarPos('threshod', 'dst1')
    # cv2.imshow('dst1', black)
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
        # cv2.imshow('dst1',dst)
        cany = cv2.GaussianBlur(dst, (3, 3), 0)
        edges = cv2.Canny(cany, 50, 150, apertureSize=3)
        cv2.imshow('Canny', edges)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
        # lines = cv2.HoughLinesP(edges, 0.8, np.pi / 180, threshod, minLineLength, maxLineGap)
        result = dst.copy()

        theta_str =0
        str_len = 0
        rho1 = 0
        rho2 =0
        rho_center =0
        try:

            for line in lines:
                rho, theta = line[0]




                if (theta<1.3 and theta> 0.2) or(theta <2.94 and theta>1.7):
                     continue
                if theta <= 0.2 :
                     theta = 3.14-theta


                print(theta,rho)

                if rho1 == 0 and theta > 1.7:
                 rho1 = rho
                if rho-rho1 > 120 and rho2 == 0 and theta > 1.7:
                    rho2 = rho
                if rho-rho1 < -120 and rho2 ==0 and theta > 1.7:
                    rho2 = rho1
                    rho1 = rho

                if abs(rho - rho1) < 10 :
                    rho1 += rho
                    rho1 = rho1/2
                if abs(rho - rho2) < 10:
                    rho2 += rho
                    rho2 = rho2 / 2





                if theta > 1.7 :
                    theta_str += theta
                    str_len +=1
            theta_str = theta_str/str_len

            if (rho1 != 0 and rho2 != 0):
                rho_center = (rho1 + rho2) / 2
            elif (rho1 !=0 and rho2 ==0) :
                rho_center = rho1
            elif (rho1 == 0 and rho2 !=0):
                rho_center = rho2



            a = np.cos(theta_str)
            b = np.sin(theta_str)
            x0 = a * rho_center
            y0 = b * rho_center
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)

            deltax = np.cos(3.14-theta_str)*rho_center - 256
            delta_theta = 3.14 - theta_str

        except:
            pass
        # for line in lines:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow('dst1', result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        a=a+1
        # print(a)
        continue
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
