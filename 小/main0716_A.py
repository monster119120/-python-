#coding:utf-8
import numpy as np
import cv2
import time
import serial
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
ser.open()

cap1 = cv2.VideoCapture(0)
cap1.set(5,120)
cap1.set(3,160)
cap1.set(4,120)
# cap2 = cv2.VideoCapture(0)
# cap2.set(5,120)
# cap3 = cv2.VideoCapture(2)
# cap3.set(5,120)

# cv2.namedWindow('ohyeah',cv2.WINDOW_NORMAL)
# cv2.namedWindow('edges',cv2.WINDOW_NORMAL)
# cv2.namedWindow('img',cv2.WINDOW_NORMAL)
# cv2.namedWindow('newimg',cv2.WINDOW_NORMAL)
# cv2.namedWindow('erosion',cv2.WINDOW_NORMAL)
# DIM=(640, 480)
# K=np.array([[397.4669696815286, 0.0, 317.35864489667904], [0.0, 397.71007137527664, 227.52377178271067], [0.0, 0.0, 1.0]])
# D=np.array([[-0.09019384973015974], [0.00609795762227383], [-0.4084158097868423], [0.9724865927597655]])

DIM=(160, 120)
K=np.array([[99.4258503276217, 0.0, 78.82468853503748], [0.0, 99.48655709062508, 56.50370696954958], [0.0, 0.0, 1.0]])
D=np.array([[-0.054589320778828936], [-0.38118622492409726], [1.1750597946154482], [-1.256404940962829]])

stickin = time.time()
hengzhe = [1,1,1]
shuzhe = [1,1,1]
get_heng = 0
get_shu = 0
get_shu_pass , get_heng_pass = 0,0
hengxianloc,hengxianlocup,hengxianlocdown = 50,50,50
shuxianloc ,shuxianlocup,shuxianlocdown= 45,45,45
shifoupandingflag = 1
jiansuflag = 0
danciget = 0  # 当一条线被监测到了的时候，另一条线需要检测
lineleave_heng , lineleave_shu = 0,0
kuandu = 10
baifenbi = 0.835
deltaheng, deltahengflag,  deltashu, deltashuflag, thetaheng, thetahengflag, thetashu , thetashuflag= 0.0, 0,0.0, 0,0.0, 0,0.0, 0
shiziflag = 0
step = [[3+1, 4+1]]
for teststep in step:
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
    kuandu = 10
    baifenbi = 0.944
    deltaheng, deltahengflag, deltashu, deltashuflag, thetaheng, thetahengflag, thetashu, thetashuflag = 0.0, 0, 0.0, 0, 0.0, 0, 0.0, 0
    shiziflag = 0
    ser.write(  str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(0) + '1314' + '\0')
    stickin = time.time()
    while(1):
        ret, img = cap1.read()
        (lower, upper) = ([0, 0, 120], [255, 255, 255])
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(img, lower, upper)
        img = cv2.bitwise_and(img, img, mask=mask)

        img = cv2.resize(img, DIM)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # cv2.imshow('img', img)
        cv2.imshow('img', img)
        cv2.imshow('imgcopy',img[0:80,0:100])

        newimg = cv2.resize(img[0:80,0:100],(100,90))
        # cv2.imshow('newimg',newimg)
        img = cv2.cvtColor(newimg, cv2.COLOR_BGR2GRAY)
        wtf, img = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
        # cv2.imshow('erosion',cv2.erode(img,np.ones((15,15),np.uint8),iterations = 1))
        # img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 5, 10)
        cany = cv2.GaussianBlur(img, (13, 13), 0)
        edges = cv2.Canny(cany, 50, 70, apertureSize=3)
        # cv2.imshow('edges',edges)
        # ret2 , img2 = cap2.read()
        # ret3, img3 = cap3.read()
        undistorted_img = img
        if img[50-kuandu:50+kuandu,0:90].sum()/255 + img[0:100,45-kuandu:45+kuandu].sum()/255 - (2*kuandu*90 - 2*kuandu*100)*baifenbi >= 0 :
            shiziflag = 1
        else:
            shiziflag = 0




# ————————————————————————————————————————————————————————————————————
        shuzhesao = undistorted_img.sum(axis=0)/255 # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSSS
        shuzhesao1 = np.where(shuzhesao >= 80, 1, 0)
        believeshuzhe = np.array([])  # 把数组每相邻个8个加起来
        for i in range(shuzhesao1.shape[0]-8+1):
            believeshuzhe = np.append(believeshuzhe, shuzhesao1[i:i+8].sum())
        shuxianloc = np.average(np.where(believeshuzhe >= 7))

        shuzhesaoup = undistorted_img[0:50].sum(axis=0) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSSs
        shuzhesao1up = np.where(shuzhesaoup >= 40, 1, 0)
        believeshuzheup = np.array([])  # 把数组每相邻个五个加起来
        for i in range(shuzhesao1up.shape[0] - 8 + 1):
            believeshuzheup = np.append(believeshuzheup, shuzhesao1up[i:i + 8].sum())
        shuxianlocup = np.average(np.where(believeshuzheup >= 7))

        shuzhesaodown = undistorted_img[50:100].sum(axis=0) / 255  # 图像竖着加和压扁SSSSSSSSSSSSSSSSSS
        shuzhesao1down = np.where(shuzhesaodown >= 40, 1, 0)
        believeshuzhedown = np.array([])  # 把数组每相邻个五个加起来
        for i in range(shuzhesao1down.shape[0] - 8 + 1):
            believeshuzhedown = np.append(believeshuzhedown, shuzhesao1down[i:i + 8].sum())
        shuxianlocdown = np.average(np.where(believeshuzhedown >= 7))
        # print(believeshuzhedown,believeshuzhe,believeshuzheup)
        # print(believeshuzheup)
        shuzhe[0] = shuzhe[1]
        shuzhe[1] = shuzhe[2]
        if np.amax(believeshuzhe)>0:
            shuzhe[2] = 1
        else:
            shuzhe[2] = 0
        if shuzhe[0] == 1 and shuzhe[1] == 1 and shuzhe[2] == 0 and shifoupandingflag == 1:
            get_shu_pass = get_shu_pass + 1
            get_shu = get_shu_pass
        if img[0:100, 45-kuandu:45+kuandu].sum()/255 - (2*kuandu*100)*baifenbi >= 0 and shifoupandingflag == 2:
            get_shu = get_shu+1
# *** *** *** ***
# ————————————————————————————————————————————————————————————————


        hengzhesao = undistorted_img.sum(axis = 1)/255 # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
        hengzhesao1 = np.where(hengzhesao >= 70, 1, 0)
        believehengzhe = np.array([]) # 把数组每相邻个五个加起来
        for i in range(hengzhesao1.shape[0]-8+1):
            believehengzhe = np.append(believehengzhe, hengzhesao1[i:i+8].sum())
        hengxianloc = np.average(np.where(believehengzhe >= 7))

        hengzhesaoup = undistorted_img[0:100, 45:90].sum(axis=1) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
        hengzhesao1up = np.where(hengzhesaoup >= 35, 1, 0)
        believehengzheup = np.array([])  # 把数组每相邻个五个加起来
        for i in range(hengzhesao1up.shape[0] - 8 + 1):
            believehengzheup = np.append(believehengzheup, hengzhesao1up[i:i + 8].sum())
        hengxianlocup = np.average(np.where(believehengzheup >= 7))

        hengzhesaodown = undistorted_img[0:100, 0:45].sum(axis=1) / 255  # 图像横着加和压扁SSSSSSSSSSSSSSSSSSS
        hengzhesao1down = np.where(hengzhesaodown >= 35, 1, 0)
        believehengzhedown = np.array([])  # 把数组每相邻个五个加起来
        for i in range(hengzhesao1down.shape[0] - 8 + 1):
            believehengzhedown = np.append(believehengzhedown, hengzhesao1down[i:i + 8].sum())
        hengxianlocdown = np.average(np.where(believehengzhedown >= 7))
        hengzhe[0] = hengzhe[1]
        hengzhe[1] = hengzhe[2]
        if np.amax(believehengzhe)>0:
            hengzhe[2] = 1
        else:
            hengzhe[2] = 0
        if hengzhe[0] == 1 and hengzhe[1] == 1 and hengzhe[2] == 0 and shifoupandingflag == 1:
            get_heng_pass = get_heng_pass + 1
            get_heng = get_heng_pass
        if img[50-kuandu:50+kuandu, 0:90].sum()/255 - (2*kuandu*90)*baifenbi >= 0 and shifoupandingflag == 2:
            get_heng = get_heng+1
        # print(shuxianlocdown,shuxianloc,shuxianlocup,hengxianlocdown,hengxianloc,hengxianlocup)
        # print(shuxianlocdown,hengxianloc)
        # cv2.imshow('ohyeah', undistorted_img)

        deltaheng = hengxianloc - 50
        deltashu = shuxianloc - 45
        if deltaheng <= 0:
            deltahengflag = 1
        else:
            deltahengflag = 0
        if deltashu <= 0:
            deltashuflag = 1
        else:
            deltashuflag = 0
        thetashu = float(shuxianlocup - shuxianlocdown)/45
        thetaheng = float(hengxianlocup - hengxianlocdown)/50
        if thetaheng >= 0:
            thetahengflag = 1
        else:
            thetahengflag = 0
        if thetashu >= 0:
            deltashuflag = 1
        else:
            thetashuflag = 0
        # print(deltaheng,deltashu,thetaheng,thetashu)
        print(get_heng,get_shu)


        if teststep[0] >= 3 and teststep[1] >= 3:
            if get_heng == teststep[0] and get_shu < teststep[1]:
                jiansuflag = 1
                shifoupandingflag = 0
                ser.write(str('x') + str(deltahengflag) + str('%05.1f' % (abs(deltaheng))) + str('t') + str(
                    thetahengflag) + str('%05.2f' % (abs(thetaheng))) + str(4) + str(0) + str(
                    jiansuflag) + '0000' + '\0')

                # 应该往横着的方向跑
                if img[0:100, 45-kuandu:45+kuandu].sum()/255 - (2*kuandu*100)*baifenbi >= 0:
                    get_shu = get_shu + 1
                if get_shu == teststep[1]:
                    shiziflag = 0
                    get_shu = 0
                    get_heng = 0
                    break

            if get_heng < teststep[0] and get_shu == teststep[1]:
                jiansuflag = 1
                shifoupandingflag = 0
                ser.write(
                    str('x') + str(deltashuflag) + str('%05.1f' % (abs(deltashu))) + str('t') + str(thetashuflag) + str(
                        '%05.2f' % (abs(thetashu))) + str(1) + str(0) + str(jiansuflag) + '0000' + '\0')
                # 应该往数着着的方向跑
                if img[50-kuandu:50+kuandu,0:90].sum()/255 - (2*kuandu*90)*baifenbi >= 0:
                    get_heng = get_heng + 1
                if get_heng == teststep[0] :
                    shiziflag = 0
                    get_shu = 0
                    get_heng = 0
                    break

            if get_heng < teststep[0]-1 and get_shu < teststep[1]-1:
                jiansuflag = 0
                shifoupandingflag = 1
                ser.write(
                    str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')

            if get_heng == teststep[0]-1 and get_shu < teststep[1]-1:
                jiansuflag = 1
                shifoupandingflag = 1
                ser.write(
                    str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')

            if get_heng < teststep[0]-1 and get_shu == teststep[1]-1:
                jiansuflag = 1
                shifoupandingflag = 1
                ser.write(
                    str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')


            if get_heng == teststep[0]-1 and get_shu == teststep[1]-1:
                jiansuflag = 1
                shifoupandingflag = 1
                ser.write(
                    str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(jiansuflag) + '1314' + '\0')

        if teststep[0] == 1 and teststep[1] > 1:
            pass
        if teststep[0] > 1 and teststep[1] == 1:
            pass
        if teststep[0] == 2 and teststep[1] > 1:
            pass
        if teststep[0] > 1 and teststep[1] == 2:
            pass
        if teststep[0] == 2 and teststep[1] == 2:
            pass


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.waitKey(1) & 0xFF == ord('Q'):
            break

ser.write(
        str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(0) + '0000' + '\0')
while(1):
    ser.write(
        str('x') + '0' + '000.0' + 't' + '0' + '00.00' + str(0) + str(0) + str(0) + '0000' + '\0')
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('Q'):
        break
cap1.release()
cv2.destroyAllWindows()

