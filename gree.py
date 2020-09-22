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
    #print("red",red_x,red_y)
    c=math.atan2(cy-red_y,cx-red_x)  
    c=int(math.degrees(c))  
    """angle_list_2 = list(range(359,0,-1))
    angle_list_2 = angle_list_2[-90:] + angle_list_2[:-90]
    angle=angle_list_2[c]"""
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
    cv2.imshow("green contour",res_g)
    cv2.waitKey(0)
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
    print(green_centers)
    g1=green_centers[0]
    c=math.atan2(cy-g1[1],cx-g1[0])  
    c=int(math.degrees(c))
   
   
    g2=green_centers[1]
    c1=math.atan2(cy-g2[1],cx-g2[0])  
    c1=int(math.degrees(c1)) 
  
   
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


def node(angle):
    node=0
    if(angle>0 and angle <30 ):
        return 1

    elif(angle>30 and angle <70 ):
        return 2

    elif(angle>70 and angle < 110):
        return 3

    elif(angle> 110 and angle <135):
        return 4

    elif(angle>135 and angle <180):
        return 5

    elif(angle>180 and angle <210):
        return 6

    elif(angle> 210 and angle < 260):
        return 7

    elif(angle>260 and angle <310):
        return 8

    elif(angle>310 and angle < 360):
        return 9

def node_difference(node1,node2):
    node=node1-node2
    if(node<0):
        node=node+9
        return node
    else:
        return node



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
    red_node=node(red_angle)
    aruco_node=node(ang)
    print("red_node",red_node)
    print("aruco_node",aruco_node)
    red_travel=node_difference(red_node,aruco_node)
    print("red node travel",red_travel)

    green_angle1,green_angle2=angle_green(frame,cx,cy)
    print("green_angles",green_angle1,green_angle2)
    green_node1=node(green_angle1)
    green_node2=node(green_angle2)
    green_travel1=node_difference(green_node1,aruco_node)
    green_travel2=node_difference(green_node2,aruco_node)

    print("g1,g2 noddes", green_node1,green_node2)

    print("g1,g2 travel", green_travel1,green_travel2)


############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()

