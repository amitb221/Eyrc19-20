import cv2
import numpy as np
import os
import math
import csv
import cv2.aruco as aruco
from aruco_lib import *
import copy
import aruco_lib as lib
import serial 

def angle_green(ip_image):
    angle = 0.00
    img=ip_image
    hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_green = np.array([10,82,161])
    upper_green = np.array([58,255,255])
    mask_g = cv2.inRange(hsv_img, lower_green, upper_green)
    res_g = cv2.bitwise_and(img,img,mask = mask_g)
    """cv2.imshow('Result_Green', res_g)
    cv2.waitKey()
    cv2.destroyAllWindows() """ 
    ret_g, thresh_g = cv2.threshold(mask_g, 127, 255, 0)
    contours_g, heirarchy = cv2.findContours(thresh_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []
    for contour in contours_g:
        approx = cv2.approxPolyDP(contour,0.001*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        #print(len(approx))
        #print(area)
        if ((len(approx)>5) &(len(approx)<21) & (area >50) & (area<80)):
            #print("green")
            #print(len(approx))
            #print(area)
            contour_list.append(contour)
            #print(len(contour_list))
    cv2.drawContours(img, contour_list,  -1, (255,0,0), 2)
    cv2.imshow("green contour",img)
    cv2.waitKey(0)
    cv2.waitKey(int(1000/32))
    green_centers=[]
    for x in contour_list:
        temp=[]
        M1=cv2.moments(x)
        green_x=int(M1["m10"]/M1["m00"])
        green_y=int(M1["m01"]/M1["m00"])
        temp.append(green_x)
        temp.append(green_y)
        green_centers.append(temp)
        print("green",green_x,green_y)  
    lower_white = np.array([0,0,230], dtype=np.uint8)
    upper_white = np.array([179,44,255], dtype=np.uint8)
    mask = cv2.inRange(hsv_img, lower_white, upper_white)
    res_w = cv2.bitwise_and(img,img,mask = mask)
    ret_w, thresh_w = cv2.threshold(mask, 127, 255, 0)
    contours_w, heirarchy = cv2.findContours(thresh_w, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_listw = []
    for contour in contours_w:
        approx = cv2.approxPolyDP(contour,0.0001*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        if ((len(approx)>6) &(len(approx)<18) & (area >64) & (area<85)):
            contour_listw.append(contour)
    for x in contour_listw:
        M1=cv2.moments(x)
        cx=int(M1["m10"]/M1["m00"])
        cy=int(M1["m01"]/M1["m00"])
    #print("white ",cx,cy)
    g1=green_centers[0]
    c=math.atan2(cy-g1[1],cx-g1[0])  
    c=math.degrees(c)  
    g2=green_centers[1]
    c1=math.atan2(cy-g2[1],cx-g2[0])  
    c1=math.degrees(c1) 
    if(c<0):
        c=180-c
    if(c1<0):
        c1=180-c1
    
    return (c,c1)



def main():
    i = 1 
    total_node=9
    cap = cv2.VideoCapture(1)
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.set(3, 640)
    cap.set(4, 480)
    ret, frame = cap.read()
    ## verifying frame has content
    #print(frame.shape)
    ## display to see if the frame is correct
    #cv2.imshow("window", frame)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    ret, frame = cap.read()
    g1,g2=angle_green(frame)
    print(g1,g2)

  

        
    
############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

