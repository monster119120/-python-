# -- coding: utf-8 --
import serial
import time
import cv2
def nothing(x):
    pass
# Create a black image, a window
img = cv2.imread('wtf.jpg')
cv2.namedWindow('image',0)
# create trackbars for color change
cv2.createTrackbar('PWM1','image',0,499,nothing)
cv2.createTrackbar('PWM2','image',0,499,nothing)
cv2.createTrackbar('PWM3','image',0,499,nothing)
cv2.createTrackbar('PWM4','image',0,499,nothing)
cv2.createTrackbar('DIR1','image',0,1,nothing)
cv2.createTrackbar('DIR2','image',0,1,nothing)
cv2.createTrackbar('DIR3','image',0,1,nothing)
cv2.createTrackbar('DIR4','image',0,1,nothing)
# cv2.createTrackbar('ENABLE','image',0,1,nothing)
cv2.createTrackbar('ROT','image',100,200,nothing)
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
ser.open()
print(ser)
print(ser.is_open)
while(1):
    try:
        cv2.imshow('image', img)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        DIR1 = cv2.getTrackbarPos('DIR1', 'image')
        DIR2 = cv2.getTrackbarPos('DIR2', 'image')
        DIR3 = cv2.getTrackbarPos('DIR3', 'image')
        DIR4 = cv2.getTrackbarPos('DIR4', 'image')
        RTO = cv2.getTrackbarPos('ROT', 'image')

        PWM1 = cv2.getTrackbarPos('PWM1', 'image')+ 500+ RTO
        PWM2 = cv2.getTrackbarPos('PWM2', 'image')+ 500- RTO
        PWM3 = cv2.getTrackbarPos('PWM3', 'image')+ 500- RTO
        PWM4 = cv2.getTrackbarPos('PWM4', 'image')+ 500+ RTO
        # DIR = cv2.getTrackbarPos('DIR', 'image') + 1
        # ENABLE = cv2.getTrackbarPos('ENABLE', 'image')
        # demo = 'A'+str(PWM1)+'B'+str(PWM2)+'C'+str(PWM3)+'D'+str(PWM4)+'W'+str(DIR)
        # ENABLE 有1和0两个值
        # demo =str(DIR1) + str(PWM1) +str(DIR2) + str(PWM2) +str(DIR3) + str(PWM3) + str(DIR4) +str(PWM4)+'\0'
        # 前1后2左3右4   左前5 右前6 右后7 左后8
        # demo = [int(PWM1),int(PWM2),int(PWM3),int(PWM4),int(DIR)]
        ser.write(str('x') + str(0) + str(0.0) + str('t') + str(0) + str(0.0) + str(1) + '\0' + str(1))
        # m=ser.readline()
        # print(m)

    except:
        pass
    # try:
    #     s = ser.readline()
    #     print(s)
    # except:
    #     pass
