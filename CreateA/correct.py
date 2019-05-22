#coding:utf-8
import cv2
import numpy as np
cv2.namedWindow('img',cv2.WINDOW_NORMAL)
cap = cv2.VideoCapture(0)
while(1):
    ret, img = cap.read()

    cv2.imshow('img', img)
    print(img.shape)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()