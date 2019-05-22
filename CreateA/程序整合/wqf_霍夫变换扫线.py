# coding:utf-8
import numpy as np
import cv2
import time
import serial
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'
ser.open()

cap = cv2.VideoCapture(0)
cap.set(5, 120)
cap.set(3, 160)
cap.set(4, 120)

DIM = (160, 120)
K = np.array([[99.4258503276217, 0.0, 78.82468853503748], [0.0, 99.48655709062508, 56.50370696954958], [0.0, 0.0, 1.0]])
D = np.array([[-0.054589320778828936], [-0.38118622492409726], [1.1750597946154482], [-1.256404940962829]])

# -------------------------------变量初始化main的-----------------------------------------
hengzhe = [1, 1, 1]
shuzhe = [1, 1, 1]
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
get, get2, get3, get4 = 0.0, 0.0, 0.0, 0.0
gettodown, gettoleft, gettoright, gettoup = 0.0, 0.0, 0.0, 0.0
stick = time.time()
second = 0.3
threshod = 50
hengxianloc, hengxianlocup, hengxianlocdown = 50, 50, 50
shuxianloc, shuxianlocup, shuxianlocdown = 45, 45, 45
danciget = 0  # 当一条线被监测到了的时候，另一条线需要检测
lineleave_heng, lineleave_shu = 0, 0
step = [[3, 4], [3, 4], [-3, -4], [3, -4], [-3, 4],
        [1, 2],  [0], [1], [2], [3],
        [3, 0],  [-3, 0], ]
angleheng, angleshu = [], []
for for_step in step:
    jiansuflag = 0
    if len(for_step) == 2:
        # -----------------------nowstep是要走的步数--------------------------
        if for_step[0] == 0 or for_step[1] == 0:
            get_heng = 0
            get_shu = 0
            DIRXIE = [0, 0, 0, 0]
            shuzhe = [1,1,1]
            hengzhe = [1,1,1]
            if for_step[0] == 0:
                nowstep = abs(for_step[1])
                if for_step[1] > 0:
                    DIR = 1
                else:
                    DIR = 2
            else:
                nowstep = abs(for_step[0])
                if for_step[0] > 0:
                    DIR = 4
                else:
                    DIR = 3
            while (1):
                try:
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
                    img = img[0:90, 0:90]
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    wtf, img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)
                    cv2.imshow('erzhi', img)
                    undistorted_img = img
                    # ############## img灰度化加二值化 # input:img     output:img
                    cany = cv2.GaussianBlur(img, (13, 13), 0)
                    edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                    # ############## img的canny变换  @ input:img       output:edges  img并没有变化
                    cv2.imshow('edges', edges)
                    lines = cv2.HoughLines(edges, 1, np.pi / 180, 32)
                    try:
                        for line in lines:
                            rho, theta = line[0]
                            a = np.cos(theta)
                            b = np.sin(theta)
                            angle = 180 * theta / np.pi
                            if DIR ==1 or DIR ==2:
                                if angle >= 45 and angle <= 135:
                                    continue
                                if angle >= 135:
                                    angleshu.append(angle - 180)
                                if angle <= 45:
                                    angleshu.append(angle)
                                delta.append((rho - 45 * b) / a)
                            if DIR ==3 or DIR ==4:
                                if angle <= 45 or angle >= 135:
                                    continue
                                angleheng.append(angle - 90)
                                delta.append((rho - 45 * a) / b)

                    except:
                        pass

                    for i in delta:
                        if i >= sum(delta) / len(delta):
                            delta1.append(i)
                        if i <= sum(delta) / len(delta):
                            delta2.append(i)
                    print('for i in delta')
                    if DIR == 1 or DIR == 2:
                        demox = '%05.1f'%(abs((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-45))
                        jiaodu = angleshu
                        demot = '%05.2f'%(abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-45 >= 0 :
                            xflag = 1
                        else:
                            xflag = 0
                    print('1111111111if DIR == 1 or DIR == 2')
                    if DIR == 3 or DIR == 4:
                        jiaodu = angleheng
                        demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45))
                        demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45 <= 0:
                            xflag = 1
                        else:
                            xflag = 0
                    print('11111111111111if DIR == 3 or DIR == 4')


                    if sum(jiaodu) / len(jiaodu) >= 0:
                        jiaoduflag = 1
                    else:
                        jiaoduflag = 0
                    print('if sum(jiaodu) / len(jiaodu) >= 0')
                    delta = []
                    delta1 = []
                    delta2 = []
                    jiaodu = []
                    angleheng, angleshu = [], []
                    ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(DIR) + str(
                        0) + str(jiansuflag) +'0000'+ '\0')
                    if DIR == 1 or DIR == 2:
                        if get_heng_pass < nowstep:
                            hengzhesao = undistorted_img.sum(axis=1) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
                            hengzhesao1 = np.where(hengzhesao >= 80, 1, 0)
                            believehengzhe = np.array([])  # 把数组每相邻个五个加起来
                            for i in range(hengzhesao1.shape[0] - 8 + 1):
                                believehengzhe = np.append(believehengzhe, hengzhesao1[i:i + 8].sum())
                            hengxianloc = np.average(np.where(believehengzhe >= 4))

                            hengzhe[0] = hengzhe[1]
                            hengzhe[1] = hengzhe[2]
                            if np.amax(believehengzhe) > 0:
                                hengzhe[2] = 1
                            else:
                                hengzhe[2] = 0
                            if hengzhe[0] == 1 and hengzhe[1] == 1 and hengzhe[2] == 0:
                                get_heng_pass = get_heng_pass + 1
                        if get_heng_pass == nowstep:
                            jiansuflag = 1
                            hengzhesao = undistorted_img.sum(axis=1) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
                            hengzhesao1 = np.where(hengzhesao >= 80, 1, 0)
                            believehengzhe = np.array([])  # 把数组每相邻个五个加起来
                            for i in range(hengzhesao1.shape[0] - 8 + 1):
                                believehengzhe = np.append(believehengzhe, hengzhesao1[i:i + 8].sum())
                            hengxianloc = np.average(np.where(believehengzhe >= 4))
                            if hengxianloc <= 50 and hengxianloc >= 40:
                                get_shu_pass ,get_heng_pass = 0, 0
                                print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
                                delta = []
                                delta1 = []
                                delta2 = []
                                jiaodu = []
                                break
                    print('2222222222if DIR == 1 or DIR == 2')
                    if DIR == 3 or DIR == 4:
                        if get_shu_pass < nowstep:
                            shuzhesao = undistorted_img.sum(axis=0) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSS
                            shuzhesao1 = np.where(shuzhesao >= 80, 1, 0)
                            believeshuzhe = np.array([])  # 把数组每相邻个五个加起来
                            for i in range(shuzhesao1.shape[0] - 8 + 1):
                                believeshuzhe = np.append(believeshuzhe, shuzhesao1[i:i + 8].sum())
                            shuxianloc = np.average(np.where(believeshuzhe >= 7))

                            shuzhe[0] = shuzhe[1]
                            shuzhe[1] = shuzhe[2]
                            if np.amax(believeshuzhe) > 0:
                                shuzhe[2] = 1
                            else:
                                shuzhe[2] = 0
                            if shuzhe[0] == 1 and shuzhe[1] == 1 and shuzhe[2] == 0:
                                get_shu_pass = get_shu_pass + 1
                        if get_shu_pass == nowstep:
                            jiansuflag = 1
                            shuzhesao = undistorted_img.sum(axis=0) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSS
                            shuzhesao1 = np.where(shuzhesao >= 80, 1, 0)
                            believeshuzhe = np.array([])  # 把数组每相邻个五个加起来
                            for i in range(shuzhesao1.shape[0] - 8 + 1):
                                believeshuzhe = np.append(believeshuzhe, shuzhesao1[i:i + 8].sum())
                            shuxianloc = np.average(np.where(believeshuzhe >= 7))
                            print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                            print(shuxianloc)


                            if shuxianloc <= 50 and shuxianloc >= 40:
                                get_shu_pass, get_heng_pass = 0, 0

                                delta = []
                                delta1 = []
                                delta2 = []
                                jiaodu = []
                                break
                    print('222222222DIR == 3 or DIR == 4')

                    print( str(get_heng_pass) + 'get_heng_pass')

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    if cv2.waitKey(1) & 0xFF == ord('Q'):
                        break
                except:
                    pass
        else:
            DIR = 0
            get_heng = 0
            get_shu = 0
            DIRXIE[0], DIRXIE[1], DIRXIE[2], DIRXIE[3] = \
                for_step[0]/abs(for_step[0]), abs(for_step[0]), \
                for_step[1] / abs(for_step[1]), abs(for_step[1]) # DIRXIE [-3,4]   0 3 1 4
            hengzhe = [1, 1, 1]
            shuzhe = [1, 1, 1]
            jiansuflag = 0
            while (1):
                try:
                    ret, img = cap.read()
                    result = img.copy()
                    (lower, upper) = ([0, 0, 120], [255, 255, 255])
                    lower = np.array(lower, dtype="uint8")
                    upper = np.array(upper, dtype="uint8")
                    mask = cv2.inRange(img, lower, upper)
                    img = cv2.bitwise_and(img, img, mask=mask)
                    # ############## img的色彩提取 @ input:img     output:img
                    img = cv2.resize(img, DIM)
                    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
                    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    # ############## img的畸变矫正 @ input:img   output:img
                    threcopy = img[0:100, 0:90]
                    threcopy = cv2.cvtColor(threcopy, cv2.COLOR_BGR2GRAY)
                    wtf, threcopy = cv2.threshold(threcopy, 100, 255, cv2.THRESH_BINARY)
                    cany = cv2.GaussianBlur(threcopy, (9, 9), 0)
                    edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                    cv2.imshow('edges', edges)
                    undistorted_img = threcopy
                    lines = cv2.HoughLines(edges, 1, np.pi / 180, 39) # 此处为滤掉所有的短线
                    try:
                        for line in lines:
                            rho, theta = line[0]
                            a = np.cos(theta)
                            b = np.sin(theta)
                            angle = 180 * theta / np.pi
                            if DIR ==1 or DIR ==2:
                                if angle >= 45 and angle <= 135:
                                    continue
                                if angle >= 135:
                                    angleshu.append(angle - 180)
                                if angle <= 45:
                                    angleshu.append(angle)
                                delta.append((rho - 45 * b) / a)
                            if DIR ==3 or DIR ==4:
                                if angle <= 45 or angle >= 135:
                                    continue
                                angleheng.append(angle - 90)
                                delta.append((rho - 45 * a) / b)
                    except:
                        pass
                    # print('len(delta)'+str(len(delta))+'len(angleshu()'+str(len(angleshu)))

                    hengzhesao = undistorted_img.sum(axis=1) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
                    hengzhesao1 = np.where(hengzhesao >= 75, 1, 0)
                    believehengzhe = np.array([])  # 把数组每相邻个五个加起来
                    for i in range(hengzhesao1.shape[0] - 8 + 1):
                        believehengzhe = np.append(believehengzhe, hengzhesao1[i:i + 8].sum())
                    hengxianloc = np.average(np.where(believehengzhe >= 4))

                    hengzhe[0] = hengzhe[1]
                    hengzhe[1] = hengzhe[2]
                    if np.amax(believehengzhe) > 0:
                        hengzhe[2] = 1
                    else:
                        hengzhe[2] = 0
                    if hengzhe[0] == 0 and hengzhe[1] == 0 and hengzhe[2] == 1:
                        get_heng_pass = get_heng_pass + 1
#########################################################################################################################################
                    shuzhesao = undistorted_img.sum(axis=0) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
                    shuzhesao1 = np.where(shuzhesao >= 75, 1, 0)
                    believeshuzhe = np.array([])  # 把数组每相邻个五个加起来
                    for i in range(shuzhesao1.shape[0] - 8 + 1):
                        believeshuzhe = np.append(believeshuzhe, shuzhesao1[i:i + 8].sum())
                    shuxianloc = np.average(np.where(believeshuzhe >= 4))
                    shuzhe[0] = shuzhe[1]
                    shuzhe[1] = shuzhe[2]
                    if np.amax(believeshuzhe) > 0:
                        shuzhe[2] = 1
                    else:
                        shuzhe[2] = 0
                    if shuzhe[0] == 0 and shuzhe[1] == 0 and shuzhe[2] == 1:
                        get_shu_pass = get_shu_pass + 1
# //////////////////////////////////////////////////横竖都在之外 都没看到/////////////////////////////////////////////////////////


                    if get_shu_pass - abs(for_step[0]) == 0 and (get_heng_pass - abs(for_step[1]) <= -1 or \
                        (hengxianloc < 25 and get_heng_pass == abs(for_step[1])))and  (shuxianloc <= 65 and shuxianloc >= 25):
                        print('11111111111111111111111111111111')
                        if DIRXIE[2] == 1:
                            DIR = 1
                        else:
                            DIR = 2


                        for i in delta:
                            if i >= sum(delta) / len(delta):
                                delta1.append(i)
                            if i <= sum(delta) / len(delta):
                                delta2.append(i)
                        print('len(delta)' + str(len(delta)) + 'len(delta1)' + str(len(delta1)) + 'len(delta2)' + str(
                            len(delta2))+'len(angleshu)'+str(len(angleshu)))

                        if DIR == 1 or DIR == 2:
                            demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45))
                            jiaodu = angleshu
                            demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                            if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45 >= 0:
                                xflag = 1
                            else:
                                xflag = 0

                        if sum(jiaodu) / len(jiaodu) >= 0:
                            jiaoduflag = 1
                        else:
                            jiaoduflag = 0
                        # print("jiaodu的len"+str(len(jiaodu)))
                        print(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(
                            DIR) + str(
                            0) + str(jiansuflag) + '0000' + '\0')
                        ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(
                            DIR) + str(
                            0) + str(jiansuflag) + '0000' + '\0')
                        delta = []
                        delta1 = []
                        delta2 = []
                        jiaodu = []
                        angleheng, angleshu = [], []
                    elif get_heng_pass - abs(for_step[1]) == 0 and (get_shu_pass - abs(for_step[0]) <= -1 or \
                        (shuxianloc < 25 and get_shu_pass == abs(for_step[0])))and (hengxianloc <= 65 and hengxianloc >= 25):
                        print('22222222222222222222')
                        if DIRXIE[0] == 1:
                            DIR = 4
                        else:
                            DIR = 3
                        for i in delta:
                            if i >= sum(delta) / len(delta):
                                delta1.append(i)
                            if i <= sum(delta) / len(delta):
                                delta2.append(i)

                        if DIR == 3 or DIR == 4:
                            demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45))
                            jiaodu = angleheng
                            demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                            if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45 <= 0:
                                xflag = 1
                            else:
                                xflag = 0
                        if sum(jiaodu) / len(jiaodu) >= 0:
                            jiaoduflag = 1
                        else:
                            jiaoduflag = 0
                        ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(
                            DIR) + str(
                            0) + str(jiansuflag) + '0000' + '\0')
                        print(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(
                            DIR) + str(
                            0) + str(jiansuflag) + '0000' + '\0')

                        delta = []
                        delta1 = []
                        delta2 = []
                        jiaodu = []
                        angleheng = []
                        angleshu = []
                    elif get_shu_pass == abs(for_step[0])and get_heng_pass == abs(for_step[1]) and \
                        (shuxianloc <= 65 and shuxianloc >= 25) and (hengxianloc <= 65 and hengxianloc >= 25):
                        print("出去了出去了除却44444444444444444444444")
                        ser.write('x0000.0t000.000000000')
                        get_shu_pass, get_heng_pass = 0, 0
                        delta = []
                        delta1 = []
                        delta2 = []
                        jiaodu = []
                        angleheng, angleshu = [], []
                        break
                    else:
                        print('4444444444444444')
                        ser.write(
                            'x0000.0t000.00000' + str(DIRXIE[0]) + str(DIRXIE[1]) + str(DIRXIE[2]) + str(DIRXIE[3]))



                    delta = []
                    delta1 = []
                    delta2 = []
                    jiaodu = []
                    angleheng, angleshu = [], []
                    if get_shu_pass - abs(for_step[0]) >= -1 or get_heng_pass - abs(for_step[1]) >= -1:
                        jiansuflag = 1
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    print('get_heng'+str(get_heng_pass)+'get_shu'+str(get_shu_pass)+'abs(for_step[0])'+str(abs(for_step[0]))+
                          'abs(for_step[1])'+str(abs(for_step[1])))
                except:
                    delta = []
                    delta1 = []
                    delta2 = []
                    jiaodu = []
                    angleheng, angleshu = [], []
                    print("cuole")
                    pass
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    if len(for_step) == 1:
        stickin = time.time()
        while(time.time()-stick<=0.5):
            ser.write('x0000.0t000.000'+str[for_step[0]]+'00000')
    stickin = time.time()
    while (time.time() - stick <= 0.1):
        ser.write('x0000.0t000.000000000')