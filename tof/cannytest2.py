import cv2
import numpy as np
cap = cv2.VideoCapture(1)
cap.set(3,1000)
cap.set(4,1000)
cap.set(5,150)
while(1):
    ret, frame = cap.read()
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('img_gray', img_gray)
    cv2.adaptiveThreshold(img_gray.copy(), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
                          cv2.THRESH_BINARY, 3, 5)
    # ret,img = cv2.threshold(haha,127,255,cv2.THRESH_BINARY)
    img = cv2.GaussianBlur(img_gray, (3, 3), 0)
    edges = cv2.Canny(img, 30, 50, apertureSize=3)
    cv2.imshow('Canny', edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()