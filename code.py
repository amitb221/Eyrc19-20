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

def angle_red(ip_image,posx,posy):
    angle = 0.00
    img=ip_image
    cx=posx
    cy=posy
    hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_red = np.array([156,125,105])
    upper_red = np.array([179,255,255])
    mask = cv2.inRange(hsv_img, lower_red, upper_red)
    res_r = cv2.bitwise_and(img,img,mask = mask)
    ret_r, thresh_r = cv2.threshold(mask, 127, 255, 0)
    contours_r, heirarchy = cv2.findContours(thresh_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []
    for contour in contours_r:
        approx = cv2.approxPolyDP(contour,0.0001*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        if((len(approx)>10)& (len(approx)<24) & (area >50) & (area <100) ):
            contour_list.append(contour)
    for x in contour_list:
        M1=cv2.moments(x)
        red_x=int(M1["m10"]/M1["m00"])
        red_y=int(M1["m01"]/M1["m00"])
    print("red",red_x,red_y)
    c=math.atan2(cy-red_y,cx-red_x)  
    c=int(math.degrees(c))  
    angle=float("{0:.2f}".format(c))
    #print(angle)
    return (180-angle)


def white(ip_image):
    img=ip_image
    hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
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
        if ((len(approx)>4) &(len(approx)<30) & (area >64) & (area<85)):
            contour_listw.append(contour)
    cv2.drawContours(img, contour_listw,  -1, (255,0,0), 2)
    for x in contour_listw:
        M1=cv2.moments(x)
        cx=int(M1["m10"]/M1["m00"])
        cy=int(M1["m01"]/M1["m00"])
    return (cx,cy)


def angle_green(ip_image,posx,posy):
    cx=posx
    cy=posy
    angle = 0.00
    img=ip_image
    hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_green = np.array([10,82,161])
    upper_green = np.array([58,255,255])
    mask_g = cv2.inRange(hsv_img, lower_green, upper_green)
    res_g = cv2.bitwise_and(img,img,mask = mask_g)
    ret_g, thresh_g = cv2.threshold(mask_g, 127, 255, 0)
    contours_g, heirarchy = cv2.findContours(thresh_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []
    for contour in contours_g:
        approx = cv2.approxPolyDP(contour,0.001*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        if ((len(approx)>5) &(len(approx)<21) & (area >50) & (area<80)):
            contour_list.append(contour)
    cv2.drawContours(img, contour_list,  -1, (255,0,0), 2)
    green_centers=[]
    for x in contour_list:
        temp=[]
        M1=cv2.moments(x)
        green_x=int(M1["m10"]/M1["m00"])
        green_y=int(M1["m01"]/M1["m00"])
        temp.append(green_x)
        temp.append(green_y)
        green_centers.append(temp)
    #print("green",green_x,green_y)  
   # print(green_centers)
    g1=green_centers[0]
    c=math.atan2(cy-g1[1],cx-g1[0])  
    c=math.degrees(c)  
    g2=green_centers[1]
    c1=math.atan2(cy-g2[1],cx-g2[0])  
    c1=math.degrees(c1) 
    c=180-c
    c1=180-c1
    return (c,c1)


def angle_aruco(ip_image):
    img=ip_image
    final=img
    list1=detect_Aruco(final)
    state=calculate_Robot_State(final,list1)
    for i in state:
        x=state.get(i)
        break
    return x[-1]


def node(ang1,ang2):
    angle=ang1-ang2
    node=round(angle/40)
    if(node<0):
        return (node+9)
    else:
        return(node)



def main():
    cap = cv2.VideoCapture(1)
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.set(3, 640)
    cap.set(4, 480)
    ret, frame = cap.read()
    #print(frame.shape)
    ret, frame = cap.read()
    ang = angle_aruco(frame)
    print("auruco_angle",ang)
    h=round(frame.shape[0] /2)
    w=round(frame.shape[1] /2)
    img=frame[h-50:h+50,w-50:w+50]
    x,y=white(img)
    posy=h-50
    posx=w-50
    cy=y+posy
    cx=x+posx
    #print(cx,cy)
    red_angle=angle_red(frame,cx,cy)
    print("Actual_red_angle",red_angle)
    #green1_angle,green2_angle=angle_green(frame,cx,cy)
    #print("Actual green angles",green1_angle,green2_angle)
    #fianl_angle_red_node=node(red_angle,ang)
    #print("final_red node",fianl_angle_red_node)
    #final_angle_green1_node=node(green1_angle,ang)
    #final_angle_green2_node=node(green2_angle,ang)
    #print("final green nodes",final_angle_green1_node,final_angle_green2_node)
   
 
  


############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

