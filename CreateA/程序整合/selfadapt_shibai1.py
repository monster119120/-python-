# coding:utf-8
import numpy as np
import cv2
import serial
import time

deltaheng, deltashu, theta = [0.0], [0.0], [0.0]
dx, dy, dt, dxflag, dyflag, dtflag = 0.0, 0.0, 0.0, 0, 0, 0

def rotateadapt(image, angle, center=None, scale=1.0): #1
    (h, w) = image.shape[:2] #2
    if center is None: #3
        center = (w // 2, h // 2) #4

    M = cv2.getRotationMatrix2D(center, angle, scale) #5

    rotated = cv2.warpAffine(image, M, (w, h)) #6
    return rotated #7
# def guoxianadapt(img, axiss,baixiankuandu , zhe,returnwaht = 1,panduantheta = 0,shouldloc = 0):
def guoxianadapt(img, axiss,baixiankuandu , zhe,returnwaht ,panduantheta ,shouldloc ):
    if str(shouldloc) == 'nan':
        return np.nan
    sao = img.sum(axis=axiss) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSSS
    sao1 = np.where(sao >= baixiankuandu, 1, 0)
    believe = np.array([])  # 把数组每相邻个8个加起来
    for i in range(sao1.shape[0] - 3 + 1):
        believe = np.append(believe, sao1[i:i + 3].sum())
    if panduantheta == 0:
        loc = np.average(np.where(believe >= 1))
    else:
        linshi = []
        for j in np.where(believe >= 1):
            for k in j:
                if abs(k - shouldloc)<=25:
                    linshi.append(k)
        if len(linshi) == 0:
            loc = np.nan
        else:
            loc = sum(linshi)/len(linshi)

    zhe[0] = zhe[1]
    zhe[1] = zhe[2]
    if np.amax(believe) > 0:
        zhe[2] = 1
    else:
        zhe[2] = 0
    if returnwaht == 1:
        return loc,zhe
    else:
        return loc



hengzheadapt = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
shuzheadapt = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
locadapt = [0, 0, 0, 0, 0, 0]
hengxianlocadapt = 0.0
shuxianlocadapt = 0.0




# ---------------------------------以上是adapt定义的变量
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
ser.open()

cap = cv2.VideoCapture(1)
cap.set(5, 120)
cap.set(3, 160)
cap.set(4, 120)

DIM = (160, 120)
K = np.array([[99.4258503276217, 0.0, 78.82468853503748], [0.0, 99.48655709062508, 56.50370696954958], [0.0, 0.0, 1.0]])
D = np.array([[-0.054589320778828936], [-0.38118622492409726], [1.1750597946154482], [-1.256404940962829]])

# -------------------------------变量初始化main的-----------------------------------------
hengzhe = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
shuzhe = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
loc = [0, 0, 0, 0]
get_heng = 0
get_shu = 0
get_shu_pass, get_heng_pass = 0, 0
shifoupandingflag = 1
jiansuflag = 0
kuandu = 6
baifenbi = 0.8
deltaheng, deltahengflag, deltashu, deltashuflag = 0.0, 0, 0.0, 0
thetaheng, thetahengflag, thetashu, thetashuflag = 0.0, 0, 0.0, 0
shiziflag = 0
stickin = time.time()
# --------------------------变量初始化testpro2的----------------------------------------
angle = 0
delta = []  # 位置偏差的所有部分的集合
delta1 = []  # 位置偏差的左边部分的集合
delta2 = []  # 位置偏差的右边部分的集合
jiaodu = []  # 角度偏差的集合
jiaoduflag = 0  # 角度的数值政府标志
xflag = 0  # 位置偏差的数值正负标志
DIR = 1  # 代表方向 1前 2后 3左 4右
DIRXIE = [0, 0, 0, 0]  # flag 数字 flag 数字 1正0负
nowstep = 0  # 当前步所需步数
jiancexian = 0  # 当前所检测到线的总条数
demox = 0.0  # 距离偏差
demot = 0.0  # 角度偏差
stick = time.time()
while(time.time() <= stick + 3):
    pass
second = 0.3
threshod = 50
hengxianloc =  50
shuxianloc =  45
erzhi1 = 20
mohu = 3
# step = [[0, -6], [-4, 0], [-3, -4], [3, -4], [-3, 4],
#         [1, 2],  [0], [1], [2], [3],
#         [3, 0],  [-3, 0], ]
step = [[-1,-2],[6,6,6,6]]
angleheng, angleshu = [], []
stick = time.time()
while(time.time()-stick<=3):
    pass
erzhi1 = 8
mohu = 3
dxlast1, dylast1, dtlast1, dxflaglast1, dyflaglast1, dtflaglast1 = 0.0, 0.0, 0.0, 0, 0, 0
dxlast2, dylast2, dtlast2, dxflaglast2, dyflaglast2, dtflaglast2 = 0.0, 0.0, 0.0, 0, 0, 0
stick = time.time()
stickinin = time.time()
while (1):
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
        img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)        # ############## img的畸变矫正 @ input:img   output:img
        resultup = img[0:60, 0:120]
        resultleft = img[0:160, 0:60]
        cv2.imshow('resultupwhite',resultup)
        cv2.imshow('resultleftwhite', resultleft)
        resultup = cv2.cvtColor(resultup, cv2.COLOR_BGR2GRAY)
        resultup = cv2.GaussianBlur(resultup, (mohu, mohu), 0)
        wtf, resultup = cv2.threshold(resultup, erzhi1, 255, cv2.THRESH_BINARY)
        # canyup = cv2.GaussianBlur(resultup, (mohu, mohu), 0)

        resultleft = cv2.cvtColor(resultleft, cv2.COLOR_BGR2GRAY)
        resultleft = cv2.GaussianBlur(resultleft, (mohu, mohu), 0)
        wqf, resultleft = cv2.threshold(resultleft, erzhi1, 255, cv2.THRESH_BINARY)
        # canyleft = cv2.GaussianBlur(resultleft, (mohu, mohu), 0)
        cv2.imshow('resultup', resultup)
        cv2.imshow('resultleft', resultleft)




        # edgesup = cv2.Canny(canyup, 50, 70, apertureSize=3)
        # cv2.imshow('edgesup',edgesup)
        # edgesleft = cv2.Canny(canyleft, 50, 70, apertureSize=3)
        # cv2.imshow('canyleft',canyleft)
        # linesup = cv2.HoughLines(edgesup, 1, np.pi / 180, 20)
        # linesleft = cv2.HoughLines(edgesleft, 1, np.pi / 180, 20)
        #print (len(linesup),len(linesleft))
        # try:
        #     for line in linesup:
        #         rho, jiaodu = line[0]
        #         a = np.cos(jiaodu)
        #         b = np.sin(jiaodu)
        #         angle = 180 * jiaodu / np.pi
        #         if angle >= 135:
        #             theta.append(angle - 180)
        #         if angle <= 45:
        #             theta.append(angle)
        #         # if angle >= 135 or angle <= 45:
        #         #     deltashu.append(abs(rho))
        #
        # except:
        #     print('upcuole')


        locadapt[1], hengzheadapt[1] = guoxianadapt(resultleft, 1,58 ,  hengzheadapt[1],1,0,0)
        locadapt[2], hengzheadapt[2] = guoxianadapt(rotateadapt(resultleft, 8), 1,58, hengzheadapt[2],1,0,0)
        locadapt[3], hengzheadapt[3] = guoxianadapt(rotateadapt(resultleft, -8), 1,58, hengzheadapt[3],1,0,0)
        locadapt[4], hengzheadapt[4] = guoxianadapt(rotateadapt(resultleft, 16), 1,58, hengzheadapt[4],1,0,0)
        locadapt[5], hengzheadapt[5] = guoxianadapt(rotateadapt(resultleft, -16), 1,58, hengzheadapt[5],1,0,0)



        finalloc = []
        locadapt[0] = [locadapt[1], locadapt[2], locadapt[3], locadapt[4], locadapt[5]]
        for i in locadapt[0]:
            if str(i) == 'nan':
                continue
            else:
                finalloc.append(i)
        if len(finalloc) == 0:
            finalloc = np.nan
        else:
            finalloc = sum(finalloc) / len(finalloc)
        hengxianlocadapt = finalloc

        locadapt[1], shuzheadapt[1] = guoxianadapt(resultup, 0, 58,shuzheadapt[1],1,0,0)
        locadapt[2], shuzheadapt[2] = guoxianadapt(rotateadapt(resultup, 8), 0,58, shuzheadapt[2],1,0,0)
        locadapt[3], shuzheadapt[3] = guoxianadapt(rotateadapt(resultup, -8), 0,58, shuzheadapt[3],1,0,0)
        locadapt[4], shuzheadapt[4] = guoxianadapt(rotateadapt(resultup, 16), 0,58, shuzheadapt[4],1,0,0)
        locadapt[5], shuzheadapt[5] = guoxianadapt(rotateadapt(resultup, -16), 0,58, shuzheadapt[5],1,0,0)

        finalloc = []
        locadapt[0] = [locadapt[1], locadapt[2], locadapt[3], locadapt[4], locadapt[5]]
        for i in locadapt[0]:
            if str(i) == 'nan':
                continue
            else:
                finalloc.append(i)
        if len(finalloc) == 0:
            finalloc = np.nan
        else:
            finalloc = sum(finalloc) / len(finalloc)
        shuxianlocadapt = finalloc

        locthetaup = guoxianadapt(resultup[0:10, 0:120], 0, 10, [1,1,1], 0, 1, shuxianlocadapt)
        locthetadown = guoxianadapt(resultup[50:60, 0:120], 0, 10,[1,1,1], 0, 1, shuxianlocadapt)
        if str(locthetaup) == 'nan' or str(locthetadown) == 'nan':
            pass
        elif locthetaup == locthetadown:
            pass
        else:
            if np.arctan(30 / (locthetaup - locthetadown)) * 180 / np.pi>=0:
                theta.append(-(90 - np.arctan(40 / (locthetaup - locthetadown)) * 180 / np.pi))
            else:
                theta.append(90 + np.arctan(40 / (locthetaup - locthetadown)) * 180 / np.pi)

        locthetaup = guoxianadapt(rotateadapt(resultup[0:10, 0:120],-8), 0, 10, [1, 1, 1], 0, 1, shuxianlocadapt)
        locthetadown = guoxianadapt(rotateadapt(resultup[50:60, 0:120],-8), 0, 10, [1, 1, 1], 0, 1, shuxianlocadapt)
        if str(locthetaup) == 'nan' or str(locthetadown) == 'nan':
            pass
        elif locthetaup == locthetadown:
            pass
        else:
            if np.arctan(30 / (locthetaup - locthetadown)) * 180 / np.pi >= 0:
                theta.append(-(90 - np.arctan(40 / (locthetaup - locthetadown)) * 180 / np.pi+8))
            else:
                theta.append(90 + np.arctan(40 / (locthetaup - locthetadown)) * 180 / np.pi+8)

        locthetaup = guoxianadapt(rotateadapt(resultup[0:10, 0:120], 8), 0, 10, [1, 1, 1], 0, 1, shuxianlocadapt)
        locthetadown = guoxianadapt(rotateadapt(resultup[50:60, 0:120], 8), 0, 10, [1, 1, 1], 0, 1, shuxianlocadapt)
        if str(locthetaup) == 'nan' or str(locthetadown) == 'nan':
            pass
        elif locthetaup == locthetadown:
            pass
        else:
            if np.arctan(30 / (locthetaup - locthetadown)) * 180 / np.pi >= 0:
                theta.append(-(90 - np.arctan(40 / (locthetaup - locthetadown)) * 180 / np.pi-8))
            else:
                theta.append(90 + np.arctan(40 / (locthetaup - locthetadown)) * 180 / np.pi-8)

        # try:
        #     for line in linesleft:
        #         rho, jiaodu = line[0]
        #         a = np.cos(jiaodu)
        #         b = np.sin(jiaodu)
        #         angle = 180 * jiaodu / np.pi
        #         if angle > 45 and angle < 135:
        #             deltaheng.append(abs(rho))
        #             theta.append(angle - 90)
        #
        # except:
        #     print('cuole')

        try:
            linshi2 = dt
            if str(dt) == 'nan':
                dt = dtlast1
                print('wrongdt')
            else:
                dt = sum(theta) / len(theta)
            dtlast1 = dt
            dtlast2 = dtlast1
        except:
            dt = dtlast1
            print('wrongdt')
        if sum(theta) >= 0:
            dtflag = 0
        else:
            dtflag = 1

        try:

            dx = shuxianlocadapt - 70
            if dx == -70 or str(dx) == 'nan':
                dx = dxlast1
            else:
                dxlast1 = dx
                dxlast2 = dxlast1
            if dx >= 0:
                dxflag = 0
            else:
                dxflag = 1
        except:
            dx = dxlast1
            dxflag = 0
            print('wrongdx')

        try:
            dy = hengxianlocadapt - 68
            if dy == -68 or str(dy) == 'nan':
                dy = dylast1
            else:
                dylast1 = dy
                dylast2 = dylast1
            if dy >= 0:
                dyflag = 0
            else:
                dyflag = 1
        except:
            dy = dylast1
            dyflag = 0
            print('wrongdy')

        dx = '%05.1f' % abs(dx/2)
        dy = '%05.1f' % abs(dy/2)
        dt = '%05.2f' % abs(dt/2)
        ser.write(
            'x' + str(dxflag) + str(dx) + 't' + str(dtflag) + str(dt) + '500' + '0000' + 'y' + str(dyflag) + str(dy))
        ser.write(0x00)
        time.sleep(0.03)
        print('x' + str(dxflag) + str(dx) + 't' + str(dtflag) + str(dt) + '500' + '0000' + 'y' + str(dyflag) + str(dy))
    except:
        print ('paofeile')
        pass

    if (abs(float(dx)) <= 4 and abs(float(dy)) <= 4 and abs(float(dt)) <= 2 and time.time()-stick>=0.1) and time.time() - stickinin >=88880:
        break
    # print('abs(float(dx))',abs(float(dx)) ,'abs(float(dy))',abs(float(dy)),'abs(float(dt))',abs(float(dt)),'locthetaup',locthetaup,'locthedown',locthetadown)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
stick = time.time()
while(time.time() - stick <= 100):
    ser.write('x0000.0t000.000000000y0000.0')
    ser.write(0x00)
    time.sleep(0.09)
    print ('overover')