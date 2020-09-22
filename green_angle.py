import cv2
import numpy as np
import os
import math
import csv
import copy

def angle_red(ip_image,posx,posy):
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
    #cv2.imshow("green contour",res_g)
    #cv2.waitKey(0)
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
    c=int(math.degrees(c))
    angle_list_2 = list(range(359,0,-1))
    angle_list_2 = angle_list_2[-90:] + angle_list_2[:-90]
    angle=angle_list_2[c]
    c=angle
    g2=green_centers[1]
    c1=math.atan2(cy-g2[1],cx-g2[0])  
    c1=int(math.degrees(c1)) 
    angle1=angle_list_2[c1]
    c1=angle1
    c=180-c
    c1=180-c1
    return (c,c1)

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


