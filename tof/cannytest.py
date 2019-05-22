# -- coding: utf-8 --
import cv2
import numpy as np
cap = cv2.VideoCapture(1)
cap.set(5,120)
cap.set(5,120)
cap.set(3,160)
i = 3
print(cap.get(15))
DIM=(160, 120)
K=np.array([[99.4258503276217, 0.0, 78.82468853503748], [0.0, 99.48655709062508, 56.50370696954958], [0.0, 0.0, 1.0]])
D=np.array([[-0.054589320778828936], [-0.38118622492409726], [1.1750597946154482], [-1.256404940962829]])

# print(cap.get(0),cap.get(1),cap.get(2),cap.get(3),cap.get(4),cap.get(5),cap.get(6),cap.get(7),cap.get(8),cap.get(9),cap.get(10),cap.get(11),cap.get(12),cap.get(13),cap.get(14),cap.get(15),cap.get(16),cap.get(17),cap.get(18),cap.get(19),cap.get(30))
while(1):
    ret, Img = cap.read()
    img = cv2.resize(Img, DIM)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    Img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    image = Img
    (lower, upper) =([0, 0, 130], [255, 255, 255])
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)
    output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', output)
    img = cv2.GaussianBlur(output, (17, 17), 0)
    edges = cv2.Canny(img, 50, 70, apertureSize=3)
    cv2.imshow('edges', edges)
    wtf, output = cv2.threshold(output, 100, 255, cv2.THRESH_BINARY)
    cv2.imshow('threshold',output)
    cv2.imshow('some',output[0:100,0:90])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('w'):
        cv2.imwrite('picsaidao\\wqfwqf'+str(i)+'.png',Img)
        i = i + 1

cap.release()
cv2.destroyAllWindows()