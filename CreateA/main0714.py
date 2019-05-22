# -- coding: utf-8 --
import cv2
import numpy as np
import time
import serial
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
ser.open()

cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
cap.set(5,120)
print(cap.get(3),cap.get(4)) #640 480
angle = 0
delta = []#位置偏差的所有部分的集合
delta1 = []#位置偏差的左边部分的集合
delta2 = []#位置偏差的右边部分的集合
jiaodu = []#角度偏差的集合
jiaoduflag = 0#角度的数值政府标志
xflag = 0#位置偏差的数值正负标志
# qizi = [[0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0],
#         [0,   1,  0,  1,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0],
#         [1,   1,  1,  0,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0]]
# final = []
# final = trans(route_plan(qizi))
DIR = 0 #代表方向 1前 2后 3左 4右
nowstep = 0 #当前步所需步数
jiancexian = 0 #当前所检测到线的总条数
demox = 0.0 #距离偏差
demot = 0.0 #角度偏差
get = 0.0
get1 = 0.0
get2 = 0.0
get3 =0.0
gettodown, gettoleft, gettoright, gettoup = 0.0, 0.0, 0.0, 0.0
stick = time.time()
second = 0.7
threshod = 80
start = [0,0]
debug = 0
shifoujiansu = 0


for step in [[],[],[],[],[]]:
    if len(step) == 2:
        linshi = [step[0]-start[0],step[1]-start[1]]
        start = [step[0],step[1]]
        step = linshi
        if step == [0,0]:
            continue
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
        while(1):
            try:
                ret, frame = cap.read()
                (lower, upper) = ([0, 0, 78], [255, 255, 255])
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
                cany = cv2.GaussianBlur(outputdst, (17, 17), 0)
                wtf, threcopy = cv2.threshold(outputdst, 100, 255, cv2.THRESH_BINARY)
                cv2.imshow('output',output)
                cv2.imshow('outputdst',outputdst)
                get = (threcopy[191-75:191+75].sum(axis = 0)[456:476].sum()+threcopy[48-15:68-15].sum(axis=0)[203:283+80].sum())/255- (160*20+150*20)*0.8895
                gettoup = (threcopy[191-75-20:191+75-20].sum(axis = 0)[456:476].sum()+threcopy[48-15:68-15].sum(axis=0)[203:283+80].sum())/255- (160*20+150*20)*0.866
                gettodown = (threcopy[191-75+20:191+75+20].sum(axis = 0)[456:476].sum()+threcopy[48-15:68-15].sum(axis=0)[203:283+80].sum())/255- (160*20+150*20)*0.866
                gettoleft = (threcopy[191-75:191+75].sum(axis = 0)[456:476].sum()+threcopy[48-15:68-15].sum(axis=0)[203-65:283+80-65].sum())/255- (160*20+150*20)*0.866
                gettoright = (threcopy[191-75:191+75].sum(axis = 0)[456:476].sum()+threcopy[48-15:68-15].sum(axis=0)[203+65:283+80+65].sum())/255- (160*20+150*20)*0.866
                # get1 = (threcopy[168:312].sum(axis = 0)[246+21:640].sum()+threcopy[0:312].sum(axis=0)[246+21:400+21].sum())/255- (154*312+144*375)*0.968
                # get2 = (threcopy[168:312].sum(axis = 0)[:].sum()+threcopy[:].sum(axis=0)[246+21:400+21].sum())/255- (154*480+144*640)*0.9233
                # get3 = (threcopy[168:312].sum(axis = 0)[:].sum()+threcopy[0:312].sum(axis=0)[246+21:400+21].sum())/255- (154*312+144*640)*0.968
                print('eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee')
                edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                cv2.imshow('edges',edges)
                print('huofu000000000000')
                lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod) # 此处为滤掉所有的短线
                print('huofu11111111111111111111111111')
                result = dst.copy()
                try:
                    for line in lines:
                        rho, theta = line[0]
                        a = np.cos(theta)
                        b = np.sin(theta)
                        # x0 = a * rho
                        # y0 = b * rho
                        # x1 = int(x0 + 1000 * (-b))
                        # y1 = int(y0 + 1000 * (a))
                        # x2 = int(x0 - 1000 * (-b))
                        # y2 = int(y0 - 1000 * (a))
                        angle = 180 * theta / np.pi
                        if DIR == 1 or DIR ==2 :
                            if angle >= 45 and angle <= 135:
                                continue
                            if angle >= 135:
                                angle = angle - 180
                            jiaodu.append(angle)
                            delta.append((rho - 192 * b) / a)
                        if DIR == 3 or DIR == 4:
                            if angle <= 45 or angle >= 135:
                                continue
                            angle = angle - 90
                            jiaodu.append(angle)
                            delta.append((rho - 283 * a) / b)
                except:
                    print("cuolexian")
                    pass
                for i in delta:
                    if i >= sum(delta) / len(delta):
                        delta1.append(i)
                    if i <= sum(delta) / len(delta):
                        delta2.append(i)
                # print (len(delta1), len(delta2), len(delta), len(jiaodu))
                print('littlerui')
                if DIR == 1 or DIR == 2:
                    demox = '%05.1f'%(abs((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283))
                    demot = '%05.2f'%(abs(sum(jiaodu) / len(jiaodu)))
                    if (sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283 >= 0 :
                        xflag = 1
                    else:
                        xflag = 0
                if DIR == 3 or DIR == 4:
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
                print('ruiruiruiruiuriuuirui')
                if DIR == 1:
                    ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) +str(demot) + str(1) + str(2)+str(shifoujiansu)+'\0')
                if DIR == 2:
                    ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(2)+ str(2)+str(shifoujiansu) + '\0')
                if DIR == 3:
                    ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(4) + str(2)+str(shifoujiansu)+ '\0')
                if DIR == 4:
                    ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(3)+ str(2)+str(shifoujiansu) + '\0')
                delta = []
                delta1 = []
                delta2 = []
                jiaodu = []
                print('ddddddddddddddddddddddddddddddddddddd')
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                print("cuole")
                pass
            if ( get>=0 or gettoleft>=0 or gettodown>=0 or gettoright >=0 or gettoup >=0 )  and time.time()-stickin >= second:
                stickin = time.time()
                jiancexian = jiancexian + 1
            # print(jiancexian,nowstep)
            # print()
            if jiancexian == nowstep:
                debug = debug + 1
                print(debug)

                jiancexian = 0
                break
    if len(step) == 1 and step[0] == 1:#1是拿起，0是放下
        stick = time.time()
        shifoujiansu = 0
        while (1):
            # print(time.time(), jiancexian)
            if time.time() - stick <= 1.0:
                ser.write(str('x') + str(0) + str(111.1) + str('t') + str(1) + str(11.11) + str(0)+str(1) +str(shifoujiansu)+ '\0')
                stickin = time.time()
            else:
                break
    if len(step) == 1 and step[0] == 0:
        stick = time.time()
        shifoujiansu = 0
        while (1):
            # print(time.time(), jiancexian)
            if time.time() - stick <= 1.0:
                ser.write(str('x') + str(0) + str(111.1) + str('t') + str(1) + str(11.11) + str(0) +str(0)+str(shifoujiansu)+ '\0')
                stickin = time.time()
            else:
                break
    if len(step) == 3:
        shifoujiansu = 1
        # ser.write(str('x') + str(0) + str(111.1) + str('t') + str(1) + str(11.11) + str(0) + str(0) +str(1)+str(shifoujiansu)+ '\0')
    # stick = time.time()
    # while(1):
    #     if time.time()-stick <= 0.5:
    #         ser.write(str('x') + str(0) + str(111.1) + str('t') + str(0) + str(11.11) + str(0) + str(3)+'\0')
    #
    #         stickin=time.time()
    #     else:
    #         break
    # stickin = time.time()