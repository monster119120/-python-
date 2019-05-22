# -- coding: utf-8 --
while (1):
    # minLineLength = cv2.getTrackbarPos('minLineLength', 'dst1')
    # maxLineGap = cv2.getTrackbarPos('maxLineGap', 'dst1')
    # threshod = cv2.getTrackbarPos('threshod', 'dst1')
    try:
        ret, frame = cap.read()
        (lower, upper) = ([0, 0, 100], [255, 255, 255])
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(frame, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask=mask)
        output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        wtf, output = cv2.threshold(output, 100, 255, cv2.THRESH_BINARY)
        img_gray = frame
        # 摄像头畸变矫正
        dst = camerajiaozheng2()
        # dst为畸变矫正后的灰度图像
        # cv2.imshow('dst1',dst)
        maskdst = cv2.inRange(dst, lower, upper)
        outputdst = cv2.bitwise_and(dst, dst, mask=maskdst)
        outputdst = cv2.cvtColor(outputdst, cv2.COLOR_BGR2GRAY)
        cany = cv2.GaussianBlur(outputdst, (3, 3), 0)
        threcopy = output
        # print(get)
        get = (threcopy[168:312].sum(axis=0)[246:640].sum() + threcopy[0:312].sum(axis=0)[246:400].sum()) / 255 - (
                    154 * 312 + 160 * 400) * 0.8
        edges = cv2.Canny(cany, 50, 70, apertureSize=3)
        # cv2.imshow('output', output)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshod)
        result = dst.copy()
        try:
            for line in lines:
                rho, theta = line[0]
                angle = 180 * theta / np.pi
                # if angle >= 70 and angle <= 110:
                #     xian.append(1)
                if angle <= 45 or angle >= 135:
                    continue
                angle = angle - 90
                jiaodu.append(angle)
                a = np.cos(theta)
                b = np.sin(theta)
                delta.append((rho - 283 * a) / b)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                # cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)
        # cv2.imshow('result', result)
        except:
            print("cuolexian")
            pass
        for i in delta:
            if i >= sum(delta) / len(delta):
                delta1.append(i)
            else:
                delta2.append(i)
        # print((sum(delta1)/len(delta1)+sum (delta2)/len(delta2))/2-283)
        # print(sum(jiaodu) / len(jiaodu))
        print (len(delta1), len(delta2), len(delta), len(jiaodu))
        demox = '%05.1f' % (abs((sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 192))
        demot = '%05.2f' % (abs(sum(jiaodu) / len(jiaodu)))
        if (sum(delta1) / len(delta1) + sum(delta2) / len(delta2)) / 2 - 192 >= 0:
            xflag = 1
        else:
            xflag = 0
        if sum(jiaodu) / len(jiaodu) >= 0:
            jiaoduflag = 1
        else:
            jiaoduflag = 0
        ser.write(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(3) + '\0')
        # ser.write( str('t')+ str(demot)+"\0")
        # print(demox,demot)
        # print(str('x') + str(xflag) + str(demox) + str('t') + str(jiaoduflag) + str(demot) + str(3) + '\0')
        # cv2.imshow('dst1', result)
        delta = []
        delta1 = []
        delta2 = []
        jiaodu = []
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        print("cuole")
        pass
    # if max(xian) == -1:
    #     jiancexianforfor = jiancexianfor
    #     jiancexianfor = jiancexiannow
    #     jiancexiannow = 0
    # else:
    #     jiancexianforfor = jiancexianfor
    #     jiancexianfor = jiancexiannow
    #     jiancexiannow = 1
    # print(xian)
    # xian = [-1]
    if get >= 0 and time.time() - stickin >= second:
        stickin = time.time()
        jiancexian = jiancexian + 1
    # print(jiancexian,get)
    if jiancexian == nowstep:
        jiancexian = 0
        break