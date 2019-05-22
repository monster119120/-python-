import cv2
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
cap.set(5,120)
print(cap.get(3))
i = 0
while(1):
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('img_gray', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('w'):
        cv2.imwrite('pic150\\pic'+str(i)+'.png',frame)
        i = i + 1
cap.release()
cv2.destroyAllWindows()
