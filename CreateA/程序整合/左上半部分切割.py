# coding:utf-8
import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)
cap.set(5, 120)
cap.set(3, 160)
cap.set(4, 120)

DIM = (160, 120)
K = np.array([[99.4258503276217, 0.0, 78.82468853503748], [0.0, 99.48655709062508, 56.50370696954958], [0.0, 0.0, 1.0]])
D = np.array([[-0.054589320778828936], [-0.38118622492409726], [1.1750597946154482], [-1.256404940962829]])

hengzhe = [1, 1, 1]
shuzhe = [1, 1, 1]
get_heng = 0
get_shu = 0
get_shu_pass, get_heng_pass = 0, 0
hengxianloc, hengxianlocup, hengxianlocdown = 50, 50, 50
shuxianloc, shuxianlocup, shuxianlocdown = 45, 45, 45
shifoupandingflag = 1
jiansuflag = 0
danciget = 0  # 当一条线被监测到了的时候，另一条线需要检测
lineleave_heng, lineleave_shu = 0, 0
kuandu = 6
baifenbi = 0.8
deltaheng, deltahengflag, deltashu, deltashuflag, thetaheng, thetahengflag, thetashu, thetashuflag = 0.0, 0, 0.0, 0, 0.0, 0, 0.0, 0
shiziflag = 0
teststep = [3 + 1, 4 + 1]
stickin = time.time()

while (1):
    ret, img = cap.read()
    (lower, upper) = ([0, 0, 130], [255, 255, 255])
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")
    mask = cv2.inRange(img, lower, upper)
    img = cv2.bitwise_and(img, img, mask=mask)
    # ############## img的色彩提取 @ input:img     output:img
    img = cv2.resize(img, DIM)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # ############## img的畸变矫正 @ input:img   output:img
    cv2.imshow('img', img)
    cv2.imshow('imgshu', img[0:160, 0:90])
    cv2.imshow('imheng', img[0:90, 0:120])
    img = img[0:90, 0:90]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    wtf, img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)
    cv2.imshow('erzhi', img)
    # ############## img灰度化加二值化 # input:img     output:img
    cany = cv2.GaussianBlur(img, (13, 13), 0)
    edges = cv2.Canny(cany, 50, 70, apertureSize=3)
    # ############## img的canny变换  @ input:img       output:edges  img并没有变化
    cv2.imshow('edges', edges)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('Q'):
        break