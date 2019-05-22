# coding:utf-8
import numpy as np
import cv2
import serial
import time
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
ser.open()

cap = cv2.VideoCapture(1)
cap.set(5, 120)
cap.set(3, 160)
cap.set(4, 120)

DIM = (160, 120)
K = np.array([[99.4258503276217, 0.0, 78.82468853503748], [0.0, 99.48655709062508, 56.50370696954958], [0.0, 0.0, 1.0]])
D = np.array([[-0.054589320778828936], [-0.38118622492409726], [1.1750597946154482], [-1.256404940962829]])

stick = time.time()
while(time.time() - stick <=3):
    try:
        jiansuflag = 2
        deltaheng, deltashu, theta = [0.0], [0.0], [0.0]
        dx, dy, dt, dxflag, dyflag, dtflag = 0.0, 0.0, 0.0, 0, 0, 0
        ret, img = cap.read()
        (lower, upper) = ([0, 0, 70], [255, 255, 255])
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(img, lower, upper)
        img = cv2.bitwise_and(img, img, mask=mask)
        # ############## img的色彩提取 @ input:img     output:img
        img = cv2.resize(img, DIM)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # ############## img的畸变矫正 @ input:img   output:img
        resultup = img[0:60, 0:120]
        resultleft = img[0:160, 0:60]

        resultup = cv2.cvtColor(resultup, cv2.COLOR_BGR2GRAY)
        wtf, resultup = cv2.threshold(resultup, 120, 255, cv2.THRESH_BINARY)
        canyup = cv2.GaussianBlur(resultup, (7, 7), 0)
        resultleft = cv2.cvtColor(resultleft, cv2.COLOR_BGR2GRAY)
        wqf, resultleft = cv2.threshold(resultleft, 20, 255, cv2.THRESH_BINARY)
        canyleft = cv2.GaussianBlur(resultleft, (7, 7), 0)


        cv2.imshow('canyup', canyup)
        cv2.imshow('canyleft', canyleft)

        edgesup = cv2.Canny(canyup, 50, 70, apertureSize=3)
        edgesleft = cv2.Canny(canyleft, 50, 70, apertureSize=3)

        linesup = cv2.HoughLines(edgesup, 1, np.pi / 180, 20)
        linesleft = cv2.HoughLines(edgesleft, 1, np.pi / 180, 20)

        try:
            for line in linesup:
                rho, jiaodu = line[0]
                a = np.cos(jiaodu)
                b = np.sin(jiaodu)
                angle = 180 * jiaodu / np.pi
                if angle >= 135:
                    theta.append(angle - 180)
                if angle <= 45:
                    theta.append(angle)
                if angle>= 135 or angle<=45:
                    deltashu.append(abs(rho))
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(resultup, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.imshow('resultup', resultup)
                print('up')
        except:
            pass
        try:
            for line in linesleft:
                rho, jiaodu = line[0]
                a = np.cos(jiaodu)
                b = np.sin(jiaodu)
                angle = 180 * jiaodu / np.pi
                if angle > 45 and angle <135:
                    deltaheng.append(abs(rho))
                    theta.append(angle - 90)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(resultleft, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.imshow('resultleft', resultleft)
                print('left')
        except:
            pass
        print('11111')
        print(len(theta))
        try:
            dt = sum(theta)/len(theta)
        except:
            dt = 0.0
            pass
        print('222222')
        print(len(theta))
        if sum(theta)>=0 :
            dtflag = 1
        else:
            dtflag = 0




        try:
            dx = sum(deltashu)/len(deltashu) - 80
            if dx>= 0:
                dxflag = 0
            else:
                dxflag = 1
        except:
            dx = 0.0
            dxflag = 0
            pass
        print('33333333')
        try:
            dy = sum(deltaheng)/len(deltaheng) - 60
            if dy>= 0:
                dyflag = 0
            else:
                dyflag = 1
        except:
            dy = 0.0
            dyflag = 0
            pass
        dx = '%05.1f'%abs(dx)
        dy = '%05.1f'%abs(dy)
        dt = '%05.2f'%abs(dt)
        ser.write('x'+str(dxflag)+str(dx)+'t'+str(dtflag)+str(dt)+'500'+'0000'+'y'+str(dyflag)+str(dy))
        print('gogogo')
    except:
        pass



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
stick = time.time()
while(time.time() - stick <= 0.5):
    ser.write('x0000.0t000.000000000y0000.0')
    print ('overover')