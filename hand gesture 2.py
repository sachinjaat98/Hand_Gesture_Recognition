#hand gesture with pyautogui
import cv2 as cv
import numpy as np
import math
#import pyautogui


cap = cv.VideoCapture(0)
kernal = np.ones((5, 5), np.uint8)

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


        #drawing contours
        #cv.drawContours(drawing,[max_cnt],-1,(0,255,0),1
        #cv.drawContours(drawing,[hull],-1,(255,255,0),2)



        # displaying number of fingers
        if count ==0:
            if  area_cnt <2000:
                #pyautogui.moveTo(1200, 300)
                cv.putText(frame,"put hand in the box",(20,50),cv.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            else:

                #pyautogui.dragRel(300,0,1)
                cv.putText(frame,"closed hand",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)

        elif count ==4:
            cv.putText(frame,"open hand",(20,50),cv.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2)

    except:
        pass

    cv.imshow("frame", frame)
    cv.imshow("thresh",thresh)

    if cv.waitKey(1) & 0xff == 13:
        break

cap.release()
cv.destroyAllWindows()

pyautogui.displayMousePosition()