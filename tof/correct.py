# -- coding: utf-8 --
from getroute import *
import numpy as np
import serial
import cv2
import glob
import time
stickin = time.time()
# 摄像头畸变矫正函数，传入图像，出来矫正系数
def camerajiaozheng1(s='pic150\\*.png'):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    objpoints = []
    imgpoints = []
    images = glob.glob(s)
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    return ret, mtx, dist, rvecs, tvecs
ret,mtx,dist,rvecs,tvecs=camerajiaozheng1()
# ret,mtx,dist,rvecs,tvecs = 0.164165913373,np.array([[684.02167337,0.0,361.31274026],[0.0,683.56601082,235.27341269],[0.0 ,0.0,1.0]]),np.array([[-0.48922554, -1.48290388 ,-0.01576108, -0.02200881  ,7.25511877]]),[[ 0.37417391],[-0.03346785],[-1.54973317]],[[-4.01649789],[ 2.1647331 ],[15.21416375]]
def camerajiaozheng2():
    h, w = img_gray.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
    dst = cv2.remap(img_gray, mapx, mapy, cv2.INTER_LINEAR)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    return dst
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
ser.open()
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
cap.set(5,120)
print(cap.get(3),cap.get(4)) #640 480

# qizi = [[0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0],
#         [0,   1,  0,  1,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0],
#         [1,   1,  1,  0,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0]]

while(1):
    try:
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        img_gray = frame
        # 摄像头畸变矫正
        dst = camerajiaozheng2()
        # dst为畸变矫正后的灰度图像
        cv2.imshow('dst1',dst)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        pass



cap.release()
cv2.destroyAllWindows()