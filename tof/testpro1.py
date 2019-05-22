# -- coding: utf-8 --
import getroute
from getroute import route_plan
from getroute import trans
import numpy as np
import serial
import cv2
import glob
import time
stickin = time.time()
# 摄像头畸变矫正函数，传入图像，出来矫正系数
# def camerajiaozheng1(s='piccam\\*.png'):
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#     objp = np.zeros((6*7,3), np.float32)
#     objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
#     objpoints = []
#     imgpoints = []
#     images = glob.glob(s)
#     for fname in images:
#         img = cv2.imread(fname)
#         gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#         ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
#         if ret == True:
#             objpoints.append(objp)
#             corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
#             imgpoints.append(corners2)
#     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
#     return ret, mtx, dist, rvecs, tvecs
# ret,mtx,dist,rvecs,tvecs=camerajiaozheng1()
ret,mtx,dist,rvecs,tvecs = 0.164165913373,np.array([[684.02167337,0.0,361.31274026],[0.0,683.56601082,235.27341269],[0.0 ,0.0,1.0]]),np.array([[-0.48922554, -1.48290388 ,-0.01576108, -0.02200881  ,7.25511877]]),[[ 0.37417391],[-0.03346785],[-1.54973317]],[[-4.01649789],[ 2.1647331 ],[15.21416375]]
def camerajiaozheng2():
    h, w = img_gray.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
    dst = cv2.remap(img_gray, mapx, mapy, cv2.INTER_LINEAR)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    return dst
# def nothing(x):
#     pass
# cv2.namedWindow('dst1')
# cv2.createTrackbar('minLineLength','dst1',20,100,nothing)
# cv2.createTrackbar('maxLineGap','dst1',20,100,nothing)
# cv2.createTrackbar('threshod','dst1',120,500,nothing)
# black = np.zeros((300, 512, 3), np.uint8)
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
ser.open()
cap = cv2.VideoCapture(1)
cap.set(5,120)
angle = 0
delta = []#位置偏差的所有部分的集合
delta1 = []#位置偏差的左边部分的集合
delta2 = []#位置偏差的右边部分的集合
jiaodu = []#角度偏差的集合
jiaoduflag = 0#角度的数值政府标志
xflag = 0#位置偏差的数值正负标志
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
# jiancexianforfor = 0
# jiancexianfor = 0 #上一次的线监测结果
# jiancexiannow = 0 # 当前的线监测结果
jiancexian = 0 #当前所检测到线的总条数
# xian = [-1] #用来暂存当前一帧图像所有的横线数目
demox = 0 #距离偏差
demot = 0 #角度偏差
get = 0.0
stick = time.time()
second = 1.5
threshod = 90
for step in [[-3,0],[0,-3],[3,0],[0,3],[-3,0],[0,-3],[3,0],[0,3],[-3,0],[0,-3],[3,0],[0,3],[-3,0],[0,-3],[3,0],[0,3],[-3,0],[0,-3],[3,0],[0,3],[-3,0],[0,-3],[3,0],[0,3],[-3,0],[0,-3],[3,0],[0,3],[-3,0],[0,-3],[3,0],[0,3],]:
# for step in [[0,6],[5,0],[0,-6],[-5,0]]:
    stickin = time.time()
    if step[0] == 0:
        nowstep = abs(step[1])
        if step[1] > 0:
            DIR = 1
        else:
            DIR = 2
    else:
        nowstep = abs(step[0])
        if step[0] > 0:
            DIR = 4
        else:
            DIR = 3
    # 判断当线的条数为 nowstep 时跳出循环
    # 当前方向为dirc，1前 2后 3左 4右
    if DIR == 1:
        while(1):
            # minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            # maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            # threshod = cv2.getTrackbarPos('threshod', 'dst1')
            try:
                ret, frame = cap.read()
                (lower, upper) = ([0, 0, 100], [255, 255, 255])
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                mask = cv2.inRange(frame, lower, upper)
                output = cv2.bitwise_and(frame, frame, mask=mask)
                output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
                wtf, output = cv2.threshold(output, 100, 255, cv2.THRESH_BINARY)
                img_gray = frame
                # 摄像头畸变矫正
                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                maskdst = cv2.inRange(dst, lower, upper)
                outputdst = cv2.bitwise_and(dst, dst, mask=maskdst)
                outputdst = cv2.cvtColor(outputdst, cv2.COLOR_BGR2GRAY)
                cany = cv2.GaussianBlur(outputdst, (3, 3), 0)
                threcopy = output
                get = (threcopy[168:312].sum(axis = 0)[246:640].sum()+threcopy[0:312].sum(axis=0)[246:400].sum())/255- (154*312+160*400)*0.8
                # print(get)
                edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                # cv2.imshow('output', output)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)#此处为滤掉所有的短线
                # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 80, 80 , 10)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        # cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        # x1,y1,x2,y2 = line
                        # cv2.line (result,(x1,y1),(x2,y2),(0,255,0),2)

                        # theta1  =  np.arctan(x1/y1)
                        # theta2  =  np.arctan(x2/y2)
                        #
                        # rho = (np.sin(theta1-theta2))*np.sqrt(x1*x1+y1*y1)*np.sqrt(x2*x2+y2*y2)/np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
                        #
                        # if(y1==y2):
                        #     theta_k = 90
                        # else :
                        #     theta_k = np.arctan((x2-x1)/(y2-y1))
                        #
                        # theta = 90 - theta_k


                        # cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        angle = 180 * theta / np.pi
                        # if angle >= 70 and angle <= 110:
                        #     xian.append(1)
                        if angle >= 45 and angle <= 135:
                            continue
                        if angle >= 135:
                            angle = angle - 180
                        jiaodu.append(angle)
                        delta.append((rho - 192 * b) / a)

                # cv2.imshow('result', result)
                except:
                    print("cuolexian")
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283)
                # print(sum(jiaodu) / len(jiaodu))
                print (len(delta1), len(delta2), len(delta), len(jiaodu))
                demox = '%05.1f'%(abs((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283))
                demot = '%05.2f'%(abs(sum(jiaodu) / len(jiaodu)))
                if (sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283 >= 0 :
                    xflag = 1
                else:
                    xflag = 0
                if sum(jiaodu) / len(jiaodu) >= 0:
                    jiaoduflag = 1
                else:
                    jiaoduflag = 0
                ser.write(str('x')+str(xflag)+str(demox)+str('t') + str(jiaoduflag)+str(demot)+str(1)+'\0')
                # ser.write( str('t')+ str(demot)+"\0")
                # print(demox,demot)
                # print(str('x')+str(xflag)+str(demox)+str('t') + str(jiaoduflag)+str(demot)+str(1)+'\0')
                # ser.write("10"+ '\0')
                # print('x'+str(demox)+'\0')

                # print('t' + str(demot) + '\0')
                # print(ser.readline())
                # time.sleep(0.05)
                # cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                print("cuole")
                pass
            # if max(xian) == -1:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 0
            # else:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 1
            # print(xian)
            # xian = [-1]

            if get >= 0 and time.time()-stickin >= second:
                stickin = time.time()
                jiancexian = jiancexian + 1

            # print(jiancexian,get)
            if jiancexian == nowstep:
                jiancexian = 0
                break
    if DIR == 2:
        while(1):
            # minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            # maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            # threshod = cv2.getTrackbarPos('threshod', 'dst1')
            try:
                ret, frame = cap.read()
                (lower, upper) = ([0, 0, 100], [255, 255, 255])
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                mask = cv2.inRange(frame, lower, upper)
                output = cv2.bitwise_and(frame, frame, mask=mask)
                output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
                wtf, output = cv2.threshold(output, 100, 255, cv2.THRESH_BINARY)
                img_gray = frame
                # 摄像头畸变矫正
                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                maskdst = cv2.inRange(dst, lower, upper)
                outputdst = cv2.bitwise_and(dst, dst, mask=maskdst)
                outputdst = cv2.cvtColor(outputdst, cv2.COLOR_BGR2GRAY)
                cany = cv2.GaussianBlur(outputdst, (3, 3), 0)
                threcopy = output
                get = (threcopy[168:312].sum(axis = 0)[246:640].sum()+threcopy[0:312].sum(axis=0)[246:400].sum())/255- (154*312+160*400)*0.8
                # print(get)
                edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                # cv2.imshow('output', output)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        angle = 180 * theta / np.pi
                        # if angle >= 70 and angle <= 110:
                        #     xian.append(1)
                        if angle >= 45 and angle <= 135:
                            continue
                        if angle >= 135:
                            angle = angle - 180
                        jiaodu.append(angle)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        delta.append((rho - 192 * b) / a)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.imshow('result', result)
                except:
                    print("cuolexian")
                    pass

                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283)
                # print(sum(jiaodu) / len(jiaodu))
                print (len(delta1),len(delta2),len(delta),len(jiaodu))
                demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 283))
                demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                print (demox,demot)
                if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 283 >= 0:
                    xflag = 1
                else:
                    xflag = 0
                if sum(jiaodu) / len(jiaodu) >= 0:
                    jiaoduflag = 1
                else:
                    jiaoduflag = 0
                ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(2) + '\0')
                # ser.write( str('t')+ str(demot)+"\0")
                # print(demox,demot)
                # print(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(2) + '\0')
                print('fffffffffffffffffff')
                # cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                print("cuole")
                pass
            # if max(xian) == -1:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 0
            # else:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 1
            # print(xian)
            # xian = [-1]

            if get >= 0 and time.time()-stickin >= second:
                stickin = time.time()
                jiancexian = jiancexian + 1
            # print(jiancexian,get)
            if jiancexian == nowstep:
                jiancexian = 0
                break
    if DIR == 3:
        while(1):
            # minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            # maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            # threshod = cv2.getTrackbarPos('threshod', 'dst1')
            try:
                ret, frame = cap.read()
                (lower, upper) = ([0, 0, 100], [255, 255, 255])
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                mask = cv2.inRange(frame, lower, upper)
                output = cv2.bitwise_and(frame, frame, mask=mask)
                output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
                wtf, output = cv2.threshold(output, 100, 255, cv2.THRESH_BINARY)
                img_gray = frame
                # 摄像头畸变矫正
                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                maskdst = cv2.inRange(dst, lower, upper)
                outputdst = cv2.bitwise_and(dst, dst, mask=maskdst)
                outputdst = cv2.cvtColor(outputdst, cv2.COLOR_BGR2GRAY)
                cany = cv2.GaussianBlur(outputdst, (3, 3), 0)
                threcopy = output
                get = (threcopy[168:312].sum(axis = 0)[246:640].sum()+threcopy[0:312].sum(axis=0)[246:400].sum())/255- (154*312+160*400)*0.8
                # print(get)
                edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        angle = 180 * theta / np.pi
                        # if angle >= 70 and angle <= 110:
                        #     xian.append(1)
                        if angle <= 45 or angle >= 135:
                            continue
                        angle = angle - 90
                        jiaodu.append(angle)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        delta.append((rho - 283 * a) / b)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        # cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # cv2.imshow('result', result)
                except:
                    print("cuolexian")
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283)
                # print(sum(jiaodu) / len(jiaodu))
                print (len(delta1), len(delta2), len(delta), len(jiaodu))
                demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 192))
                demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 192 >= 0:
                    xflag = 1
                else:
                    xflag = 0
                if sum(jiaodu) / len(jiaodu) >= 0:
                    jiaoduflag = 1
                else:
                    jiaoduflag = 0
                ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(4) + '\0')
                # ser.write( str('t')+ str(demot)+"\0")
                # print(demox,demot)
                # print(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(4) + '\0')
                # cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                print("cuole")
                pass
            # if max(xian) == -1:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 0
            # else:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 1
            # print(xian)
            # xian = [-1]

            if get >= 0 and time.time()-stickin >= second:
                stickin = time.time()
                jiancexian = jiancexian + 1
            # print(jiancexian,get)
            if jiancexian == nowstep:
                jiancexian = 0
                break
    if DIR == 4:
        while(1):
            # minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
            # maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
            # threshod = cv2.getTrackbarPos('threshod', 'dst1')
            try:
                ret, frame = cap.read()
                (lower, upper) = ([0, 0, 100], [255, 255, 255])
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                mask = cv2.inRange(frame, lower, upper)
                output = cv2.bitwise_and(frame, frame, mask=mask)
                output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
                wtf, output = cv2.threshold(output, 100, 255, cv2.THRESH_BINARY)
                img_gray = frame
                # 摄像头畸变矫正
                dst = camerajiaozheng2()
                # dst为畸变矫正后的灰度图像
                # cv2.imshow('dst1',dst)
                maskdst = cv2.inRange(dst, lower, upper)
                outputdst = cv2.bitwise_and(dst, dst, mask=maskdst)
                outputdst = cv2.cvtColor(outputdst, cv2.COLOR_BGR2GRAY)
                cany = cv2.GaussianBlur(outputdst, (3, 3), 0)
                threcopy = output
                # print(get)
                get = (threcopy[168:312].sum(axis = 0)[246:640].sum()+threcopy[0:312].sum(axis=0)[246:400].sum())/255- (154*312+160*400)*0.8
                edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                # cv2.imshow('output', output)
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        angle = 180 * theta / np.pi
                        # if angle >= 70 and angle <= 110:
                        #     xian.append(1)
                        if angle <= 45 or angle >= 135:
                            continue
                        angle = angle - 90
                        jiaodu.append(angle)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        delta.append((rho - 283 * a) / b)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        # cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # cv2.imshow('result', result)
                except:
                    print("cuolexian")
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    else:
                        delta2.append(i)
                # print((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283)
                # print(sum(jiaodu) / len(jiaodu))
                print (len(delta1), len(delta2), len(delta), len(jiaodu))
                demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 192))
                demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 192 >= 0:
                    xflag = 1
                else:
                    xflag = 0
                if sum(jiaodu) / len(jiaodu) >= 0:
                    jiaoduflag = 1
                else:
                    jiaoduflag = 0
                ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(3) + '\0')
                # ser.write( str('t')+ str(demot)+"\0")
                # print(demox,demot)
                # print(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(3) + '\0')
                # cv2.imshow('dst1', result)
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                print("cuole")
                pass
            # if max(xian) == -1:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 0
            # else:
            #     jiancexianforfor = jiancexianfor
            #     jiancexianfor = jiancexiannow
            #     jiancexiannow = 1
            # print(xian)
            # xian = [-1]
            if get >= 0 and time.time()-stickin >= second:
                stickin = time.time()
                jiancexian = jiancexian + 1
            # print(jiancexian,get)
            if jiancexian == nowstep:
                jiancexian = 0
                break
    stick = time.time()
    while(1):
        if time.time()-stick <= 0.5:
            ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(0) + '\0')

            stickin=time.time()
        else:
            break

while(1):
    ser.write(str('x')+str(xflag)+str(demox)+str('t') + str(jiaoduflag)+str(demot)+str(0)+'\0')
    print('aaaaaaaaaa')
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
