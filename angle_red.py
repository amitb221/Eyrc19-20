import cv2
import numpy as np
import os
import math
import csv
import copy

def angle_red(ip_image):
    angle = 0.00
    img=ip_image
    hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #red object detection 
    lower_red = np.array([156,125,105])
    upper_red = np.array([179,255,255])
    mask = cv2.inRange(hsv_img, lower_red, upper_red)
    res_r = cv2.bitwise_and(img,img,mask = mask)
    #cv2.imshow('Result_White', res_r)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    ret_r, thresh_r = cv2.threshold(mask, 127, 255, 0)
    contours_r, heirarchy = cv2.findContours(thresh_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []
    for contour in contours_r:
        approx = cv2.approxPolyDP(contour,0.0001*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        #print(len(approx))
        #print(area)
        if((len(approx)>10)& (len(approx)<24) & (area >50) & (area <100) ):
            #print("red")
            #print(len(approx))
            #print(area)
            contour_list.append(contour)
    cv2.drawContours(img, contour_list,  -1, (255,0,0), 2)
    cv2.imshow('red',img)
    cv2.waitKey(int(1000/32))
    for x in contour_list:
        M1=cv2.moments(x)
        red_x=int(M1["m10"]/M1["m00"])
        red_y=int(M1["m01"]/M1["m00"])
    print("red",red_x,red_y)
    #cv2.imshow("window", ip_image)
    #cv2.waitKey(0)
    #White center
    lower_white = np.array([0,0,230], dtype=np.uint8)
    upper_white = np.array([179,44,255], dtype=np.uint8)
    mask = cv2.inRange(hsv_img, lower_white, upper_white)
    res_w = cv2.bitwise_and(img,img,mask = mask)
    #cv2.imshow('Result_White', res_w)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    ret_w, thresh_w = cv2.threshold(mask, 127, 255, 0)
    contours_w, heirarchy = cv2.findContours(thresh_w, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(img, contours_w,  -1, (255,0,0), 2)
    contour_listw = []
    for contour in contours_w:
        approx = cv2.approxPolyDP(contour,0.0001*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        #print(len(approx))
        #print(area)
        if ((len(approx)>6) &(len(approx)<18) & (area >64) & (area<85)):
            #print("white")
            #print(len(approx))
            #print(area)
            contour_listw.append(contour)
    cv2.drawContours(img, contour_listw,  -1, (255,0,0), 2)
    cv2.imshow('white contour',img)
    cv2.waitKey(0)
    for x in contour_listw:
        M1=cv2.moments(x)
        cx=int(M1["m10"]/M1["m00"])
        cy=int(M1["m01"]/M1["m00"])
    print("white ",cx,cy)
    c=math.atan2(cy-red_y,cx-red_x,)  
    c=math.degrees(c)  
    angle=float("{0:.2f}".format(c))
    #text="Angle:{0}".format(angle)
    #op_image = cv2.putText(img,text,(45,45),cv2.FONT_HERSHEY_SIMPLEX ,0.5,(0,0,255),)
    #cv2.imshow("final",op_image)
    return (180-angle)


def main(): 
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(1) #if you have a webcam on your system, then change 0 to 1
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## setting the video counter to frame sequence
    cap.set(3, 640)
    cap.set(4, 480)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    #cv2.imshow("image",frame)
    #cv2.waitKey(0)
    #while(ret):
    ret, frame = cap.read()
    ## display to see if the frame is correct
    cv2.imshow("window", frame)
    cv2.waitKey(int(1000/fps));
    ## calling the algorithm function
    red_angle = angle_red(frame)
    print("red_angle",red_angle)

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()


