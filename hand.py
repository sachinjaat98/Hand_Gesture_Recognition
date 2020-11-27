import cv2 as cv
import numpy as np
import math

cap = cv.VideoCapture(0)

while True:

    ret, frame = cap.read()  # reading frame
    frame = cv.flip(frame, 1)

    cv.rectangle(frame, (400, 100), (600, 350), (0, 0, 255), 2)
    ROI = frame[100:350, 400:600]  # extracting region of interset #smoothing the roi
    hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)
    lower_b = np.array([2, 20, 70])
    upper_b = np.array([20, 255, 255])
    mask = cv.inRange(hsv, lower_b, upper_b)

    # Morphological
    kernal = np.ones((5, 5), np.uint8)
    dilate = cv.dilate(mask,kernal,iterations=2)
    erosion = cv.erode(dilate, kernal, iterations=1)

    #applying gausian and threshold
    gaussian = cv.GaussianBlur(erosion, (3,3), 0)
    ret,thresh = cv.threshold(gaussian,127,255,cv.THRESH_BINARY)

    #finding contours
    contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_NONE)



    try:
        #find contours with max area
        max_cnt = max(contours,key= cv.contourArea)

        #find convex hull
        hull = cv.convexHull(max_cnt)

        # define area of hull and area of hand
        area_hull = cv.contourArea(hull)
        area_cnt = cv.contourArea(max_cnt)

        # find the percentage of area not covered by hand in convex hull
        area_ratio = ((area_hull - area_cnt) / area_cnt) * 100
        print(area_ratio)

        #drawing contours
        #cv.drawContours(drawing,[max_cnt],-1,(0,255,0),1)
        #cv.drawContours(drawing,[hull],-1,(255,255,0),2)

        #find convexity defects
        hull = cv.convexHull(max_cnt,returnPoints= False)
        defects = cv.convexityDefects(max_cnt,hull)
        #cv.drawContours(drawing, [defects], -1, (0,0,255), 5)

        count = 0
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(max_cnt[s][0])
            end = tuple(max_cnt[e][0])
            far = tuple(max_cnt[f][0])
            pt = (100, 180)
            #calculating value sides a,b,c
            a = math.sqrt((end[0]- start[0])**2 + (end[1]- start[1])**2)
            b= math.sqrt((far[0]- start[0])**2 + (far[1]- start[1])**2)
            c= math.sqrt((end[0]- far[0])**2 + (end[1]- far[1])**2)

            # calculating angle between  ab and ac which is aplha
            angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) * 57

            #calculating area of each triangle
            s = (a + b + c) / 2
            ar = math.sqrt(s * (s - a) * (s - b) * (s - c))

            # distance between point and convex hull
            d = (2 * ar) / a

            # ignore angles > 90 and ignore points very close to convex hull(they generally come due to noise)
            if angle < 90 and d>30:
                cv.circle(ROI,far,5,(0,0,255),-1)
                count +=1
            cv.line(ROI,start,end,(0,255,0),2)

        # displaying number of fingers
        if count ==0:
            if  area_cnt <2000:
                cv.putText(frame,"put hand in the box",(20,50),cv.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            else:
                if area_ratio <10:
                    cv.putText(frame,"0",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)
                elif 11 <area_ratio <18:
                    cv.putText(frame,"dislike",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)
                elif 20 < area_ratio <30:
                    cv.putText(frame,"ONE",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)
                elif area_ratio >30:
                    cv.putText(frame,"best of luck",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)





        elif count ==1:
            if area_ratio <40:
                cv.putText(frame,"Two",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)
            else:
                cv.putText(frame, "Victory", (20, 50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)

        elif count ==2:
            if area_ratio <45:
                cv.putText(frame, "three", (20, 50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
            elif area_ratio >50:
                cv.putText(frame, "OK", (20, 50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        elif count ==3:
            cv.putText(frame,"four",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)
        elif count == 4:
            cv.putText(frame,"five",(20, 50),cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        else:
            pass
    except:
        pass

    cv.imshow("frame", frame)
    #cv.imshow("gauss",gaussian)
    cv.imshow("thresh",thresh)


    if cv.waitKey(1) & 0xff == 13:
        break

cap.release()
cv.destroyAllWindows()
