# -- coding: utf-8 --
# 读取picture中的图片，进行圆匹配
# 输出的是2*8的矩阵
# 还未进行修改


import cv2
import numpy as np
Target=cv2.imread('picpipei\\yuan1.jpg',0)
value=0.7
img_gray = cv2.imread('picpipei\\pic98.jpg',0)
def yuan_pipei(img_gray ,Target , value = 0.7 ):
    template = Target
    w, h = template.shape[::-1]
    res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
    threshold = value
    loc = np.where( res >= threshold)
    # for pt in zip(*loc[::-1]):
    #     cv2.rectangle(img_gray, pt, (pt[0] + w, pt[1] + h), (7,249,151), 2)
    #     cv2.imwrite('wah.jpg',img_gray)

    qizi = [[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]]
    for pt in zip(*loc[::-1]):
        temp0 = pt[0]/200
        temp1 = pt[1]/150
        qizi[temp0][temp1] = 1
    return qizi
qizi = yuan_pipei(img_gray,Target,value)
print(qizi)