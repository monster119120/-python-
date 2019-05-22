# -- coding: utf-8 --
import getroute
from getroute import route_plan
from getroute import trans
import copy
import numpy as np
import serial
import cv2
import glob
# 摄像头畸变矫正函数，传入图像，出来矫正系数
def camerajiaozheng1(s = 'piccam\\*.png'):
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
ret, mtx, dist, rvecs, tvecs = camerajiaozheng1()

def camerajiaozheng2():
    h, w = img_gray.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
    dst = cv2.remap(img_gray, mapx, mapy, cv2.INTER_LINEAR)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    return dst
def nothing(x):
    pass
cv2.namedWindow('dst1')
cv2.createTrackbar('minLineLength','dst1',20,100,nothing)
cv2.createTrackbar('maxLineGap','dst1',20,100,nothing)
cv2.createTrackbar('threshod','dst1',85,500,nothing)
black = np.zeros((300,512,3),np.uint8)
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
ser.open()
cap = cv2.VideoCapture(1)
angle = 0
delta = []
delta1 = []
delta2 = []
jiaodu = []


qizi = [[0,   1,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [0,   1,  0,  1,  0,  0,  0,  0],
        [0,   1,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [1,   1,  1,  0,  0,  0,  0,  0],
        [0,   1,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0]]
final = []
final = trans(route_plan(qizi))
DIR = 0 #代表方向 1前 2后 3左 4右
nowstep = 0 #当前步所需步数
jiancexianfor = 0 #上一次的线监测结果
jiancexiannow = 0 # 当前的线监测结果
jiancexian = 0 #当前所检测到线的总条数
for step in final:
    if step[0] == 0:
        nowstep = abs(step[1])
        if step[1] > 0:
            DIR = 4
        else:
            DIR = 3
    else:
        nowstep = abs(step[0])
        if step[0] > 0:
            DIR = 1
        else:
            DIR = 2
    # 判断当线的条数为 nowstep 时跳出循环
    # 当前方向为dirc，1前 2后 3左 4右
    if DIR == 1:
        while(1):
            minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            threshod = cv2.getTrackbarPos('threshod', 'dst1')

            try:
                ret, frame = cap.read()
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # 摄像头畸变矫正

                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                cany = cv2.GaussianBlur(dst, (3, 3), 0)
                edges = cv2.Canny(cany, 50, 150, apertureSize=3)
                # cv2.imshow('Canny', edges)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        angle = 180 * theta / np.pi
                        if angle>= 70 and angle <= 110:
                            jiancexianfor = jiancexiannow
                            jiancexiannow = 1
                        if angle >= 45 and angle <= 135:
                            continue
                        if angle >= 135:
                            angle = angle - 180
                        jiaodu.append(angle)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        delta.append((rho - 283 * b) / a)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # cv2.imshow('result', result)
                except:
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283)
                print(sum(jiaodu) / len(jiaodu))
                cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                pass
            if jiancexianfor == 0 and jiancexiannow ==1 :
                jiancexian = jiancexian + 1
            if jiancexian == nowstep:
                jiancexian = 0
                break
    if DIR == 2:
        while (1):
            minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            threshod = cv2.getTrackbarPos('threshod', 'dst1')

            try:
                ret, frame = cap.read()
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # 摄像头畸变矫正

                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                cany = cv2.GaussianBlur(dst, (3, 3), 0)
                edges = cv2.Canny(cany, 50, 150, apertureSize=3)
                # cv2.imshow('Canny', edges)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        angle = 180 * theta / np.pi
                        if angle >= 45 and angle <= 135:
                            continue
                        if angle >= 135:
                            angle = angle - 180
                        jiaodu.append(angle)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        delta.append((rho - 283 * b) / a)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # cv2.imshow('result', result)
                except:
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum(delta2)/len(delta2))/2-283)
                print(sum(jiaodu) / len(jiaodu))
                cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                pass
            if jiancexian == nowstep:
                break
    if DIR == 3:
        while (1):
            minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            threshod = cv2.getTrackbarPos('threshod', 'dst1')

            try:
                ret, frame = cap.read()
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # 摄像头畸变矫正

                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                cany = cv2.GaussianBlur(dst, (3, 3), 0)
                edges = cv2.Canny(cany, 50, 150, apertureSize=3)
                # cv2.imshow('Canny', edges)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        angle = 180 * theta / np.pi
                        if angle >= 45 and angle <= 135:
                            continue
                        if angle >= 135:
                            angle = angle - 180
                        jiaodu.append(angle)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        delta.append((rho - 283 * b) / a)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # cv2.imshow('result', result)
                except:
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum(delta2)/len(delta2))/2-283)
                print(sum(jiaodu) / len(jiaodu))
                cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                pass
            if jiancexian == nowstep:
                break
    if DIR == 4:
        while (1):
            minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            threshod = cv2.getTrackbarPos('threshod', 'dst1')

            try:
                ret, frame = cap.read()
                img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # 摄像头畸变矫正

                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                cany = cv2.GaussianBlur(dst, (3, 3), 0)
                edges = cv2.Canny(cany, 50, 150, apertureSize=3)
                # cv2.imshow('Canny', edges)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        angle = 180 * theta / np.pi
                        if angle >= 45 and angle <= 135:
                            continue
                        if angle >= 135:
                            angle = angle - 180
                        jiaodu.append(angle)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        delta.append((rho - 283 * b) / a)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # cv2.imshow('result', result)
                except:
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum(delta2)/len(delta2))/2-283)
                print(sum(jiaodu) / len(jiaodu))
                cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                pass
            if jiancexian == nowstep:
                break
cap.release()
cv2.destroyAllWindows()