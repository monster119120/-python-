# coding:utf-8
import numpy as np
import cv2
import time
import serial
def rotate(image, angle, center=None, scale=1.0): #1
    (h, w) = image.shape[:2] #2
    if center is None: #3
        center = (w // 2, h // 2) #4

    M = cv2.getRotationMatrix2D(center, angle, scale) #5

    rotated = cv2.warpAffine(image, M, (w, h)) #6
    return rotated #7

def guoxian(undistorted_img, axiss, zhe):
    sao = undistorted_img.sum(axis=axiss) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSSS
    sao1 = np.where(sao >= 76, 1, 0)
    believe = np.array([])  # 把数组每相邻个8个加起来
    for i in range(sao1.shape[0] - 5 + 1):
        believe = np.append(believe, sao1[i:i + 5].sum())
    loc = np.average(np.where(believe >= 4))
    zhe[0] = zhe[1]
    zhe[1] = zhe[2]
    if np.amax(believe) > 0:
        zhe[2] = 1
    else:
        zhe[2] = 0
    return loc,zhe



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
while(time.time() <= stick + 2):
    pass
second = 0.3
threshod = 50
hengxianloc =  50
shuxianloc =  45
# step = [[0, -6], [-4, 0], [-3, -4], [3, -4], [-3, 4],
#         [1, 2],  [0], [1], [2], [3],
#         [3, 0],  [-3, 0], ]
step = [[3,4],[6,6,6]]
angleheng, angleshu = [], []
for for_step in step:
    jiansuflag = 0
    if len(for_step) == 2:
        # -----------------------nowstep是要走的步数--------------------------
        if for_step[0] == 0 or for_step[1] == 0:
            get_heng = 0
            get_shu = 0
            DIRXIE = [0, 0, 0, 0]
            hengzhe = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
            shuzhe = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
            loc = [0, 0, 0, 0]
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
                    (lower, upper) = ([0, 0, 135], [255, 255, 255])
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
                    wtf, img = cv2.threshold(img, 135, 255, cv2.THRESH_BINARY)
                    # cv2.imshow('erzhi', img)
                    undistorted_img = img
                    # ############## img灰度化加二值化 # input:img     output:img
                    cany = cv2.GaussianBlur(img, (11, 11), 0)
                    edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                    # ############## img的canny变换  @ input:img       output:edges  img并没有变化
                    cv2.imshow('undistorted_img', undistorted_img)
                    lines = cv2.HoughLines(edges, 1, np.pi / 180, 38)
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
                    if DIR == 1 or DIR == 2:
                        demox = '%05.1f'%(abs((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-45))
                        jiaodu = angleshu
                        demot = '%05.2f'%(abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-45 >= 0 :
                            xflag = 0
                        else:
                            xflag = 1
                    if DIR == 3 or DIR == 4:
                        jiaodu = angleheng
                        demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45))
                        demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                        if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45 <= 0:
                            xflag = 0
                        else:
                            xflag = 1


                    if sum(jiaodu) / len(jiaodu) >= 0:
                        jiaoduflag = 1
                    else:
                        jiaoduflag = 0
                    delta = []
                    delta1 = []
                    delta2 = []
                    jiaodu = []
                    angleheng, angleshu = [], []
                    ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(DIR) + str(
                        0) + str(jiansuflag) +'0000'+ '\0')
                    if DIR == 1 or DIR == 2:
                        if get_heng_pass >= nowstep -1 :
                            jiansuflag = 1
                        else:
                            jiansuflag = 0
                        if get_heng_pass <= nowstep:
                            loc[1], hengzhe[1] = guoxian(undistorted_img, 1, hengzhe[1])
                            loc[2], hengzhe[2] = guoxian(rotate(undistorted_img, 10), 1, hengzhe[2])
                            loc[3], hengzhe[3] = guoxian(rotate(undistorted_img, -10), 1, hengzhe[3])

                            hengzhe[0][0] = hengzhe[0][1]
                            hengzhe[0][1] = hengzhe[0][2]
                            hengzhe[0][2] = max(hengzhe[1][2], hengzhe[2][2], hengzhe[3][2])

                            finalloc = []
                            loc[0] = [loc[1], loc[2], loc[3]]
                            for i in loc[0]:
                                if str(i) == 'nan':
                                    continue
                                else:
                                    finalloc.append(i)
                            if len(finalloc) == 0:
                                finalloc = np.nan
                            else:
                                finalloc = sum(finalloc) / len(finalloc)
                            hengxianloc = finalloc
                            if hengzhe[0][0] == 0 and hengzhe[0][1] == 0 and hengzhe[0][2] == 1:
                                get_heng_pass = get_heng_pass + 1

                        if get_heng_pass == nowstep and (hengxianloc<=65 and hengxianloc >= 25):
                            get_shu_pass ,get_heng_pass = 0, 0
                            delta = []
                            delta1 = []
                            delta2 = []
                            jiaodu = []
                            break

                    if DIR == 3 or DIR == 4:
                        if get_shu_pass >= nowstep -1 :
                            jiansuflag = 1
                        else:
                            jiansuflag = 0
                        if get_shu_pass <= nowstep:
                            loc[1], shuzhe[1] = guoxian(undistorted_img, 0, shuzhe[1])
                            loc[2], shuzhe[2] = guoxian(rotate(undistorted_img, 10), 0, shuzhe[2])
                            loc[3], shuzhe[3] = guoxian(rotate(undistorted_img, -10), 0, shuzhe[3])

                            shuzhe[0][0] = shuzhe[0][1]
                            shuzhe[0][1] = shuzhe[0][2]
                            shuzhe[0][2] = max(shuzhe[1][2], shuzhe[2][2], shuzhe[3][2])

                            finalloc = []
                            loc[0] = [loc[1], loc[2], loc[3]]
                            for i in loc[0]:
                                if str(i) == 'nan':
                                    continue
                                else:
                                    finalloc.append(i)
                            if len(finalloc) == 0:
                                finalloc = np.nan
                            else:
                                finalloc = sum(finalloc) / len(finalloc)
                            shuxianloc = finalloc
                            if shuzhe[0][0] == 0 and shuzhe[0][1] == 0 and shuzhe[0][2] == 1:
                                get_shu_pass = get_shu_pass + 1
                        if get_shu_pass == nowstep and (shuxianloc<=65 and shuxianloc >= 25):
                            get_shu_pass, get_heng_pass = 0, 0

                            delta = []
                            delta1 = []
                            delta2 = []
                            jiaodu = []
                            break

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
            hengzhe = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
            shuzhe = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
            loc = [0, 0, 0, 0]
            jiansuflag = 0
            while (1):
                try:
                    ret, img = cap.read()
                    result = img.copy()
                    (lower, upper) = ([0, 0, 135], [255, 255, 255])
                    lower = np.array(lower, dtype="uint8")
                    upper = np.array(upper, dtype="uint8")
                    mask = cv2.inRange(img, lower, upper)
                    img = cv2.bitwise_and(img, img, mask=mask)
                    # ############## img的色彩提取 @ input:img     output:img
                    img = cv2.resize(img, DIM)
                    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
                    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    # ############## img的畸变矫正 @ input:img   output:img
                    threcopy = img[0:90, 0:90]
                    threcopy = cv2.cvtColor(threcopy, cv2.COLOR_BGR2GRAY)
                    wtf, threcopy = cv2.threshold(threcopy, 135, 255, cv2.THRESH_BINARY)
                    cany = cv2.GaussianBlur(threcopy, (11, 11), 0)
                    edges = cv2.Canny(cany, 50, 70, apertureSize=3)
                    cv2.imshow('threcopy', threcopy)
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

                    loc[1], hengzhe[1] = guoxian(undistorted_img, 1, hengzhe[1])
                    loc[2], hengzhe[2] = guoxian(rotate(undistorted_img, 10), 1, hengzhe[2])
                    loc[3], hengzhe[3] = guoxian(rotate(undistorted_img, -10), 1, hengzhe[3])

                    hengzhe[0][0] = hengzhe[0][1]
                    hengzhe[0][1] = hengzhe[0][2]
                    hengzhe[0][2] = max(hengzhe[1][2], hengzhe[2][2], hengzhe[3][2])

                    finalloc = []
                    loc[0] = [loc[1], loc[2], loc[3]]
                    for i in loc[0]:
                        if str(i) == 'nan':
                            continue
                        else:
                            finalloc.append(i)
                    if len(finalloc) == 0:
                        finalloc = np.nan
                    else:
                        finalloc = sum(finalloc) / len(finalloc)
                    hengxianloc = finalloc
                    if hengzhe[0][0] == 0 and hengzhe[0][1] == 0 and hengzhe[0][2] == 1:
                        get_heng_pass = get_heng_pass + 1
#########################################################################################################################################
                    loc[1], shuzhe[1] = guoxian(undistorted_img, 0, shuzhe[1])
                    loc[2], shuzhe[2] = guoxian(rotate(undistorted_img, 10), 0, shuzhe[2])
                    loc[3], shuzhe[3] = guoxian(rotate(undistorted_img, -10), 0, shuzhe[3])

                    shuzhe[0][0] = shuzhe[0][1]
                    shuzhe[0][1] = shuzhe[0][2]
                    shuzhe[0][2] = max(shuzhe[1][2], shuzhe[2][2], shuzhe[3][2])

                    finalloc = []
                    loc[0] = [loc[1], loc[2], loc[3]]
                    for i in loc[0]:
                        if str(i) == 'nan':
                            continue
                        else:
                            finalloc.append(i)
                    if len(finalloc) == 0:
                        finalloc = np.nan
                    else:
                        finalloc = sum(finalloc) / len(finalloc)
                    shuxianloc = finalloc
                    if shuzhe[0][0] == 0 and shuzhe[0][1] == 0 and shuzhe[0][2] == 1:
                        get_shu_pass = get_shu_pass + 1
# //////////////////////////////////////////////////横竖都在之外 都没看到/////////////////////////////////////////////////////////


                    if get_shu_pass == abs(for_step[0]) and (get_heng_pass - abs(for_step[1]) <= -1 or \
                        (hengxianloc < 25 and get_heng_pass == abs(for_step[1])))and  (shuxianloc <= 65 and shuxianloc >= 25):
                        print('竖线先到了')
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
                                xflag = 0
                            else:
                                xflag = 1

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
                    elif get_heng_pass == abs(for_step[1])  and (get_shu_pass - abs(for_step[0]) <= -1 or \
                        (shuxianloc < 25 and get_shu_pass == abs(for_step[0])))and (hengxianloc <= 65 and hengxianloc >= 25):
                        print('横线先到了')
                        if DIRXIE[0] == 1:
                            DIR = 4
                        else:
                            DIR = 3
                        for i in delta:
                            if i >= sum(delta) / len(delta):
                                delta1.append(i)
                            if i <= sum(delta) / len(delta):
                                delta2.append(i)
                        print('len(delta)' + str(len(delta)) + 'len(delta1)' + str(len(delta1)) + 'len(delta2)' + str(
                            len(delta2)) + 'len(angleheng)' + str(len(angleheng)))
                        if DIR == 3 or DIR == 4:
                            demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45))
                            jiaodu = angleheng
                            demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
                            if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 45 <= 0:
                                xflag = 0
                            else:
                                xflag = 1
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
                        print("出去了出去了出去了出去了出去了出去了出去了出去了")
                        ser.write('x0000.0t000.000000000')
                        get_shu_pass, get_heng_pass = 0, 0
                        delta = []
                        delta1 = []
                        delta2 = []
                        jiaodu = []
                        angleheng, angleshu = [], []
                        break
                    else:
                        print('没到没到没到没到')
                        ser.write(
                            'x0000.0t000.00000' + str(DIRXIE[0]) + str(DIRXIE[1]) + str(DIRXIE[2]) + str(DIRXIE[3]))



                    delta = []
                    delta1 = []
                    delta2 = []
                    jiaodu = []
                    angleheng, angleshu = [], []
                    if get_shu_pass - abs(for_step[0]) >= -1 or get_heng_pass - abs(for_step[1]) >= -1:
                        jiansuflag = 1
                    else:
                        jiansuflag = 0
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    print('get_heng走过的横线'+str(get_heng_pass)+'get_shu走过的竖线'+str(get_shu_pass))
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
    if len(for_step) == 3:
        pass
    stickin = time.time()
    print('搞完了搞完了终于搞完了')
    while (time.time() - stick <= 50):
        ser.write('x0000.0t000.000000000')

