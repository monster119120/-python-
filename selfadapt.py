# coding:utf-8
import numpy as np
import cv2
import serial
import time

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
ser.open()

cap = cv2.VideoCapture(0)
cap.set(5, 120)
cap.set(3, 160)
cap.set(4, 120)

DIM = (160, 120)
K = np.array([[99.4258503276217, 0.0, 78.82468853503748], [0.0, 99.48655709062508, 56.50370696954958], [0.0, 0.0, 1.0]])
D = np.array([[-0.054589320778828936], [-0.38118622492409726], [1.1750597946154482], [-1.256404940962829]])

stick = time.time()
while(time.time()-stick<=2):
    pass
while(time.time()-stick<=101):
    try:
        jiansuflag = 2
        deltaheng, deltashu, theta = [0.0], [0.0], [0.0]
        dx, dy, dt, dxflag, dyflag, dtflag = 0.0, 0.0, 0.0, 0, 0, 0
        ret, img = cap.read()
        (lower, upper) = ([0, 0, 100], [255, 255, 255])
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
        wtf, resultup = cv2.threshold(resultup, 118, 255, cv2.THRESH_BINARY)
        canyup = cv2.GaussianBlur(resultup, (7, 7), 0)
        resultleft = cv2.cvtColor(resultleft, cv2.COLOR_BGR2GRAY)
        wqf, resultleft = cv2.threshold(resultleft, 130, 255, cv2.THRESH_BINARY)
        canyleft = cv2.GaussianBlur(resultleft, (7, 7), 0)


        cv2.imshow('canyup', canyup)
        cv2.imshow('canyleft', canyleft)

        edgesup = cv2.Canny(canyup, 50, 70, apertureSize=3)
        edgesleft = cv2.Canny(canyleft, 50, 70, apertureSize=3)

        linesup = cv2.HoughLines(edgesup, 1, np.pi / 180, 32)
        linesleft = cv2.HoughLines(edgesleft, 1, np.pi / 180, 32)

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
                
        except:
            print('upcuole')
        try:
            for line in linesleft:
                rho, jiaodu = line[0]
                a = np.cos(jiaodu)
                b = np.sin(jiaodu)
                angle = 180 * jiaodu / np.pi
                if angle > 45 and angle <135:
                    deltaheng.append(abs(rho))
                    theta.append(angle - 90)
               
        except:
            print('cuole')
        
        try:
            dt = sum(theta)/len(theta)
        except:
            dt = 0.0
            print('wrongdt')
            

        if sum(theta)>=0 :
            dtflag = 0
        else:
            dtflag = 1




        try:
            dx = sum(deltashu)/len(deltashu) - 50
            #print('@@@@@@@@@@@@@@dx'+str(dx))
            if dx>= 0:
                dxflag = 0
            else:
                dxflag = 1
        except:
            dx = 0.0
            dxflag = 0
            print('wrongdx')
            

        try:
            dy = sum(deltaheng)/len(deltaheng) - 60
            if dy>= 0:
                dyflag = 0
            else:
                dyflag = 1
        except:
            dy = 0.0
            dyflag = 0
            print('wrongdy')
        
        dx = '%05.1f'%abs(dx)
        dy = '%05.1f'%abs(dy)
        dt = '%05.2f'%abs(dt)
        ser.write('x'+str(dxflag)+str(dx)+'t'+str(dtflag)+str(dt)+'500'+'0000'+'y'+str(dyflag)+str(dy))
        print('x'+str(dxflag)+str(dx)+'t'+str(dtflag)+str(dt)+'500'+'0000'+'y'+str(dyflag)+str(dy))
    except:
        pass



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
stick = time.time()
while(time.time() - stick <= 0.5):
    ser.write('x0000.0t000.0000000000y0000.0')
