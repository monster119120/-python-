# coding:utf-8
import numpy as np
import cv2
import time
import serial
# --------------------------------标准初始化-----------------------------------------
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM6'
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
step = [[0, 6], [3, 4], [-3, -4], [3, -4], [-3, 4],
        [1, 2],  [0], [1], [2], [3],
        [3, 0],  [-3, 0], ]
for for_step in step:
    jiansuflag = 0
    if len(for_step) == 2:
        # -----------------------nowstep是要走的步数--------------------------
        if for_step[0] == 0 or for_step[1] == 0:
            DIRXIE = [0, 0, 0, 0]
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
            stickin = time.time()
            baifenbi = 0.845
            shifoupandingflag = 1
            while(1):
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

                    lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)  # 此处为滤掉所有的短线
                    try:
                        for line in lines:
                            rho, theta = line[0]
                            a = np.cos(theta)
                            b = np.sin(theta)
                            angle = 180 * theta / np.pi
                            if DIR == 1 or DIR == 2:
                                if angle >= 45 and angle <= 135:
                                    continue
                                if angle >= 135:
                                    angle = angle - 180
                                jiaodu.append(angle)
                                delta.append((rho - 50 * b) / a)
                            if DIR == 3 or DIR == 4:
                                if angle <= 45 or angle >= 135:
                                    continue
                                angle = angle - 90
                                jiaodu.append(angle)
                                delta.append((rho - 45 * a) / b)
                    except:

                        pass


                    ###############扫线大集合 ######################################
                    shuzhesao = undistorted_img.sum(axis=0) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSSS
                    shuzhesao1 = np.where(shuzhesao >= 75, 1, 0)
                    believeshuzhe = np.array([])  # 把数组每相邻个五个加起来
                    for i in range(shuzhesao1.shape[0] - 8 + 1):
                        believeshuzhe = np.append(believeshuzhe, shuzhesao1[i:i + 8].sum())
                    shuxianloc = np.average(np.where(believeshuzhe >= 7))
                    shuzhe[0] = shuzhe[1]
                    shuzhe[1] = shuzhe[2]
                    shuzhe[2] = np.amax(believeshuzhe)/8
                    if shuzhe[0] ==1 and shuzhe[1] == 1 and shuzhe[2] == 0 and shifoupandingflag ==1 and time.time()-stickin>=second:
                        get_shu = get_shu + 1
                    if img[0:100, 45-kuandu:45+kuandu].sum() / 255 - 2 * kuandu * 100 >= 0 and shifoupandingflag ==2:
                        get_shu = get_shu + 1

                    hengzhesao = undistorted_img.sum(axis=1) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
                    # hengzhesao1 = np.append(hengzhesao,hengzhesao,axis = 0)/255
                    # print(undistorted_img.shape,hengzhesao1.shape)
                    hengzhesao1 = np.where(hengzhesao >= 82, 1, 0)
                    believehengzhe = np.array([])  # 把数组每相邻个五个加起来
                    for i in range(hengzhesao1.shape[0] - 8 + 1):
                        believehengzhe = np.append(believehengzhe, hengzhesao1[i:i + 8].sum() )
                    hengxianloc = np.average(np.where(believehengzhe >= 7))
                    hengzhe[0] = hengzhe[1]
                    hengzhe[1] = hengzhe[2]
                    hengzhe[2] = np.amax(believehengzhe)/8
                    if hengzhe[0] == 1 and hengzhe[1] == 1 and hengzhe[2] == 0 and shifoupandingflag == 1 and time.time()-stickin>=second:
                        get_heng = get_heng + 1
                    if img[50-kuandu:50+kuandu,0:90].sum() / 255 - 2 * kuandu * 90 >= 0 and shifoupandingflag ==2:
                        get_heng = get_heng + 1










                    for i in delta:
                        if i >= sum(delta) / len(delta):
                            delta1.append(i)
                        if i <= sum(delta) / len(delta):
                            delta2.append(i)

                    if DIR == 1 or DIR == 2:
                        demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45))
                        demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45 >= 0:
                            xflag = 1
                        else:
                            xflag = 0
                    if DIR == 3 or DIR == 4:
                        demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 50))
                        demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 50 >= 0:
                            xflag = 1
                        else:
                            xflag = 0
                    if sum(jiaodu) / len(jiaodu) >= 0:
                        jiaoduflag = 1
                    else:
                        jiaoduflag = 0
                    print(demox,demot,get_shu,get_heng)
                    if DIR == 1:
                        ser.write(
                            str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(1) + str(
                                0) + str(jiansuflag) + '0000' + '\0')
                    if DIR == 2:
                        ser.write(
                            str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(2) + str(
                                2) + str(jiansuflag) + '0000' + '\0')
                    if DIR == 3:
                        ser.write(
                            str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(4) + str(
                                2) + str(jiansuflag) + '0000' +'\0')
                    if DIR == 4:
                        ser.write(
                            str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(3) + str(
                                0) + str(jiansuflag) + '0000' + '\0')
                    delta = []
                    delta1 = []
                    delta2 = []
                    jiaodu = []

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except:
                    print("cuole")
                    pass
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if DIR == 1 or DIR == 2:
                    jiancexian = get_heng
                if DIR ==3 or DIR ==4:
                    jiancexian == get_shu

                if jiancexian == nowstep - 1:
                    jiansuflag = 1
                    shifoupandingflag = 2
                if jiancexian == nowstep:
                    jiansuflag = 0
                    jiancexian = 0
                    get_heng = 0
                    get_shu = 0
                    shifoupandingflag = 1
                    break
        else:
            DIRXIE[0], DIRXIE[1], DIRXIE[2], DIRXIE[3] = \
                for_step[0]/abs(for_step[0]), abs(for_step[0]), \
                for_step[1] / abs(for_step[1]), abs(for_step[1])
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

                    lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)  # 此处为滤掉所有的短线
                    try:
                        for line in lines:
                            rho, theta = line[0]
                            a = np.cos(theta)
                            b = np.sin(theta)
                            angle = 180 * theta / np.pi
                            if DIR == 1 or DIR == 2:
                                if angle >= 45 and angle <= 135:
                                    continue
                                if angle >= 135:
                                    angle = angle - 180
                                jiaodu.append(angle)
                                delta.append((rho - 50 * b) / a)
                            if DIR == 3 or DIR == 4:
                                if angle <= 45 or angle >= 135:
                                    continue
                                angle = angle - 90
                                jiaodu.append(angle)
                                delta.append((rho - 45 * a) / b)
                    except:

                        pass

                    ###############扫线大集合 ######################################
                    shuzhesao = undistorted_img.sum(axis=0) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSSS
                    shuzhesao1 = np.where(shuzhesao >= 75, 1, 0)
                    believeshuzhe = np.array([])  # 把数组每相邻个五个加起来
                    for i in range(shuzhesao1.shape[0] - 8 + 1):
                        believeshuzhe = np.append(believeshuzhe, shuzhesao1[i:i + 8].sum())
                    shuxianloc = np.average(np.where(believeshuzhe >= 7))
                    shuzhe[0] = shuzhe[1]
                    shuzhe[1] = shuzhe[2]
                    shuzhe[2] = np.amax(believeshuzhe) / 8
                    if shuzhe[0] == 1 and shuzhe[1] == 1 and shuzhe[
                        2] == 0 and shifoupandingflag == 1 and time.time() - stickin >= second:
                        get_shu = get_shu + 1
                    if img[0:100,
                       45 - kuandu:45 + kuandu].sum() / 255 - 2 * kuandu * 100 >= 0 and shifoupandingflag == 2:
                        get_shu = get_shu + 1

                    hengzhesao = undistorted_img.sum(axis=1) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
                    # hengzhesao1 = np.append(hengzhesao,hengzhesao,axis = 0)/255
                    # print(undistorted_img.shape,hengzhesao1.shape)
                    hengzhesao1 = np.where(hengzhesao >= 82, 1, 0)
                    believehengzhe = np.array([])  # 把数组每相邻个五个加起来
                    for i in range(hengzhesao1.shape[0] - 8 + 1):
                        believehengzhe = np.append(believehengzhe, hengzhesao1[i:i + 8].sum())
                    hengxianloc = np.average(np.where(believehengzhe >= 7))
                    hengzhe[0] = hengzhe[1]
                    hengzhe[1] = hengzhe[2]
                    hengzhe[2] = np.amax(believehengzhe) / 8
                    if hengzhe[0] == 1 and hengzhe[1] == 1 and hengzhe[
                        2] == 0 and shifoupandingflag == 1 and time.time() - stickin >= second:
                        get_heng = get_heng + 1
                    if img[50 - kuandu:50 + kuandu, 0:90].sum() / 255 - 2 * kuandu * 90 >= 0 and shifoupandingflag == 2:
                        get_heng = get_heng + 1

                    for i in delta:
                        if i >= sum(delta) / len(delta):
                            delta1.append(i)
                        if i <= sum(delta) / len(delta):
                            delta2.append(i)

                    if DIR == 1 or DIR == 2:
                        demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45))
                        demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45 >= 0:
                            xflag = 1
                        else:
                            xflag = 0
                    if DIR == 3 or DIR == 4:
                        demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 50))
                        demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 50 >= 0:
                            xflag = 1
                        else:
                            xflag = 0
                    if sum(jiaodu) / len(jiaodu) >= 0:
                        jiaoduflag = 1
                    else:
                        jiaoduflag = 0
                    print(demox, demot, get_shu, get_heng)

                    delta = []
                    delta1 = []
                    delta2 = []
                    jiaodu = []
                    if abs(for_step[0]) >= 3 and abs(for_step[1]) >= 3:
                        if get_heng == for_step[0] and get_shu < for_step[1]:
                            jiansuflag = 1
                            shifoupandingflag = 0
                            # ser.write(str('x') + str(deltahengflag) + str('%05.1f' % (abs(deltaheng))) + str('t') + str(
                            #     thetahengflag) + str('%05.2f' % (abs(thetaheng))) + str(4) + str(0) + str(
                            #     jiansuflag) + '0000' + '\0')

                            # 应该往横着的方向跑
                            if img[0:100, 45 - kuandu:45 + kuandu].sum() / 255 - (2 * kuandu * 100) * baifenbi >= 0:
                                get_shu = get_shu + 1
                            if get_shu == for_step[1]:
                                shiziflag = 0
                                get_shu = 0
                                get_heng = 0
                                break

                        if get_heng < for_step[0] and get_shu == for_step[1]:
                            jiansuflag = 1
                            shifoupandingflag = 0
                            # ser.write(
                            #     str('x') + str(deltashuflag) + str('%05.1f' % (abs(deltashu))) + str('t') + str(thetashuflag) + str(
                            #         '%05.2f' % (abs(thetashu))) + str(1) + str(0) + str(jiansuflag) + '0000' + '\0')
                            # 应该往数着着的方向跑
                            if img[50 - kuandu:50 + kuandu, 0:90].sum() / 255 - (2 * kuandu * 90) * baifenbi >= 0:
                                get_heng = get_heng + 1
                            if get_heng == for_step[0]:
                                shiziflag = 0
                                get_shu = 0
                                get_heng = 0
                                break

                        if get_heng < for_step[0] - 1 and get_shu < for_step[1] - 1:
                            jiansuflag = 0
                            shifoupandingflag = 1
                            # ser.write(
                            #     str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')

                        if get_heng == for_step[0] - 1 and get_shu < for_step[1] - 1:
                            jiansuflag = 1
                            shifoupandingflag = 1
                            # ser.write(
                            #     str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')

                        if get_heng < for_step[0] - 1 and get_shu == for_step[1] - 1:
                            jiansuflag = 1
                            shifoupandingflag = 1
                            # ser.write(
                            #     str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')

                        if get_heng == for_step[0] - 1 and get_shu == for_step[1] - 1:
                            jiansuflag = 1
                            shifoupandingflag = 1
                            # ser.write(
                            #     str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')

                    if abs(for_step[0]) == 1 and abs(for_step[1]) > 1:
                        pass
                    if abs(for_step[0]) > 1 and abs(for_step[1]) == 1:
                        pass
                    if abs(for_step[0]) == 2 and abs(for_step[1]) > 1:
                        pass
                    if abs(for_step[0]) > 1 and abs(for_step[1]) == 2:
                        pass
                    if abs(for_step[0]) == 2 and abs(for_step[1]) == 2:
                        pass

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except:
                    print("cuole")
                    pass
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


    if len(for_step) == 1:
        stickin = time.time()
        while(time.time() - stickin <= 0.5):
            ser.write('x0' + '000.0' + 't0 ' + '00.00' +
                      str(0) + str(for_step[0]) + str(0) + '0000' + '\0')

    stickin = time.time()
    while(time.time()-stickin>=0.1):
        ser.write('x0000.0t000.000000000')



