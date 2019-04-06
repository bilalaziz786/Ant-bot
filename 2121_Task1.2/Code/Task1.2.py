# -*- coding: utf-8 -*-
"""
**************************************************************************
*                  E-Yantra Robotics Competition
*                  ================================
*  This software is intended to check version compatiability of open source software
*  Theme: ANT BOT
*  MODULE: Task1.2
*  Filename: Task1.2.py
*  Version: 1.0.0  
*  Date: October 31, 2018
*  
*  Author: e-Yantra Project, Department of Computer Science
*  and Engineering, Indian Institute of Technology Bombay.
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
*     
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
"""

"""
ArUco ID Dictionaries: 4X4 = 4-bit pixel, 4X4_50 = 50 combinations of a 4-bit pixel image
List of Dictionaries in OpenCV's ArUco library:
DICT_4X4_50	 
DICT_4X4_100	 
DICT_4X4_250	 
DICT_4X4_1000	 
DICT_5X5_50	 
DICT_5X5_100	 
DICT_5X5_250	 
DICT_5X5_1000	 
DICT_6X6_50	 
DICT_6X6_100	 
DICT_6X6_250	 
DICT_6X6_1000	 
DICT_7X7_50	 
DICT_7X7_100	 
DICT_7X7_250	 
DICT_7X7_1000	 
DICT_ARUCO_ORIGINAL

Reference: http://hackage.haskell.org/package/opencv-extra-0.2.0.1/docs/OpenCV-Extra-ArUco.html
Reference: https://docs.opencv.org/3.4.2/d9/d6a/group__aruco.html#gaf5d7e909fe8ff2ad2108e354669ecd17
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import aruco_lib as t
import csv
import os

img_name = ""
idno=0
matrix = []
counter = 6

def aruco_detect(path_to_image):

    '''
    you will need to modify the ArUco library's API using the dictionary in it to the respective
    one from the list above in the aruco_lib.py. This API's line is the only line of code you are
    allowed to modify in aruco_lib.py!!!
    '''
    img = cv2.imread(path_to_image)     #give the name of the image with the complete path

    id_aruco_trace = 0
    det_aruco_list = {}
    img2 = img   #separate out the Aruco image from the whole image
    det_aruco_list = t.detect_Aruco(img2,img_name)
    if det_aruco_list:

        global idno
        for key in det_aruco_list.keys():
            idno = str(key)
            break

        img3 = t.mark_Aruco(img2,det_aruco_list)
        id_aruco_trace = t.calculate_Robot_State(img3,det_aruco_list)
        print(id_aruco_trace)        
        #cv2.imshow('image',img2)
        #cv2.waitKey(0)
    '''
    Code for triggering color detection on ID detected
    '''

    color_detect(img2) 
    
    cv2.destroyAllWindows()

def color_detect(img):
    global img_name

    red1 = [0,0,200]
    red2 = [50,50,255]

    green1 = [0,170,30]
    green2 = [20,255,70]

    blue1 = [180,100,40]
    blue2 = [255,130,80]


    if(img_name == "Image1.jpg"):

        aruco_name = 'ArUco1.jpg'
        ############## for Object1 ; Green Square -> Blue boundary##############


        lower = np.array(green1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(green2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)

        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(gray,10,255,0)

        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) == 4:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (255,0,0), 25)
        M = cv2.moments(largest)
        cx1 = int(M['m10']/M['m00'])
        cy1 = int(M['m01']/M['m00'])
        
        ############## for Object2 ; Red Circle -> Green boundary##############

    
        lower = np.array(red1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(red2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)

        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) > 7:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (0,255,0), 25)

        M = cv2.moments(largest)
        cx2 = int(M['m10']/M['m00'])
        cy2 = int(M['m01']/M['m00'])

        cv2.imshow('Image1',img)
        cv2.waitKey(0)

    elif(img_name=="Image2.jpg"):

        aruco_name = 'ArUco2.jpg'
        ############## for Object1 ; Red Triangle -> Green boundary #####


        lower = np.array(red1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(red2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) ==3:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (0,255,0), 25)

        M = cv2.moments(largest)
        cx1 = int(M['m10']/M['m00'])
        cy1 = int(M['m01']/M['m00'])

        ############## for Object2 ; Blue Square -> Red boundary #####

        lower = np.array(blue1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(blue2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) ==4:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (0,0,255), 25)

        M = cv2.moments(largest)
        cx2 = int(M['m10']/M['m00'])
        cy2 = int(M['m01']/M['m00'])

        cv2.imshow('Image2',img)
        cv2.waitKey(0)
        
    elif(img_name=="Image3.jpg"):

        aruco_name = 'ArUco3.jpg'
        ############## for Object1 ; NONE #####
        cx1=cy1='None'
        ############## for Object2 ; Blue Square -> Red boundary #####

        lower = np.array(blue1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(blue2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) >7:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (0,0,255), 25)
        M = cv2.moments(largest)
        cx2 = int(M['m10']/M['m00'])
        cy2 = int(M['m01']/M['m00'])

        cv2.imshow('Image3',img)
        cv2.waitKey(0)
    
    elif(img_name=="Image4.jpg"):

        aruco_name = 'ArUco4.jpg'
        ############## for Object1 ; Green Square -> Blue boundary #####

        lower = np.array(green1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(green2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) ==4:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (255,0,0), 25)

        M = cv2.moments(largest)
        cx1 = int(M['m10']/M['m00'])
        cy1 = int(M['m01']/M['m00'])

        ############## for Object2 ; Blue Triangle -> Red boundary #####


        lower = np.array(blue1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(blue2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) ==3:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (0,0,255), 25)

        M = cv2.moments(largest)
        cx2 = int(M['m10']/M['m00'])
        cy2 = int(M['m01']/M['m00'])

        cv2.imshow('Image4',img)
        cv2.waitKey(0)
    
    elif(img_name=="Image5.jpg"):
        
        aruco_name = 'ArUco5.jpg'
        ############## for Object1 ; Red Square -> Green boundary##############

        lower = np.array(red1)    ## Convert the parameters into a form that OpenCV can understand
        upper = np.array(red2)

        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) == 4:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, (0,255,0), 25)
        M = cv2.moments(largest)
        cx1 = int(M['m10']/M['m00'])
        cy1 = int(M['m01']/M['m00'])
    
        ############## for Object2  ; NONE ##############
        cx2=cy2='None'

        cv2.imshow('Image5',img)
        cv2.waitKey(0)


    fileName = '../Images/'+str(img_name)
    cv2.imwrite(fileName,img)

    xy1 = "("+str(cx1)+":"+str(cy1)+")"
    xy2 = "("+str(cx2)+":"+str(cy2)+")"
    
    global idno
    x = [str(aruco_name),idno,str(xy1),str(xy2)]

    
    
    with open('../2121_Task1.2.csv', 'a') as file:
         writer = csv.writer(file, delimiter = ',',quoting=csv.QUOTE_ALL)
         writer.writerow(x)

############################## DONT TOUCH ABOVE ########################

def custom_detect_Aruco(img,n,C):  #returns the detected aruco list dictionary with id: corners
    aruco_list = {}
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if n == 'ARUCO' and C == 'ORIGINAL':
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

    elif n == '4':
        if C == '50':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        
        elif C == '100':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        
        elif C == '250':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

        elif C == '1000':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)

    elif n == '5':
        if C == '50':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        
        elif C == '100':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        
        elif C == '250':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)

        elif C == '1000':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

    elif n == '6':
        if C == '50':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        
        elif C == '100':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
        
        elif C == '250':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        elif C == '1000':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)

    elif n == '7':
        if C == '50':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_50)
        
        elif C == '100':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_100)
        
        elif C == '250':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)

        elif C == '1000':
            aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)

    print(aruco_dict)
    parameters = aruco.DetectorParameters_create()  #refer opencv page for clarification
    #lists of ids and the corners beloning to each id
    print(parameters)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    #corners is the list of corners(numpy array) of the detected markers. For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
    # print len(corners), corners, ids
    print(corners)
    print(len(corners))
    gray = aruco.drawDetectedMarkers(gray, corners,ids)
    # cv2.imshow('frame',gray)
    #print (type(corners[0]))
    if len(corners):    #returns no of arucos
        #print (len(corners))
        #print (len(ids))
        for k in range(len(corners)):
            temp_1 = corners[k]
            temp_1 = temp_1[0]
            temp_2 = ids[k]
            temp_2 = temp_2[0]
            aruco_list[temp_2] = temp_1
        return aruco_list

def custom_aruco_detect(path_to_image,n,C,color1,shape1,color2,shape2):

    img = cv2.imread(path_to_image)     #give the name of the image with the complete path

    id_aruco_trace = 0
    det_aruco_list = {}
    img2 = img   #separate out the Aruco image from the whole image
    det_aruco_list = custom_detect_Aruco(img2,n,C)
    if det_aruco_list:

        global idno
        for key in det_aruco_list.keys():
            idno = str(key)
            break

        img3 = t.mark_Aruco(img2,det_aruco_list)
        id_aruco_trace = t.calculate_Robot_State(img3,det_aruco_list)
        print(id_aruco_trace)        
        #cv2.imshow('image',img2)
        #cv2.waitKey(0)
    '''
    Code for triggering color detection on ID detected
    '''

    custom_color_detect(img2,color1,shape1,color2,shape2) 
    
    cv2.destroyAllWindows()

def custom_color_detect(img,color1,shape1,color2,shape2):

    red1 = [0,0,200]
    red2 = [50,50,255]

    green1 = [0,170,30]
    green2 = [20,255,70]

    blue1 = [180,100,40]
    blue2 = [255,130,80]

    global counter
    aruco_name = 'ArUco'+str(counter)+'.jpg'
    counter+=1
    
    ############## for Object1 #############
    if not color1 == "none":
        if(color1=="red"):
            lower = np.array(red1)
            upper = np.array(red2)
            boundary = [0,255,0]

        elif(color1=="blue"):
            lower = np.array(blue1)
            upper = np.array(blue2)
            boundary = [0,0,255]

        elif(color1=="green"):
            lower = np.array(green1)
            upper = np.array(green2)
            boundary = [255,0,0]

        if(shape1=="triangle"):
            sides = 3
        elif(shape1=="square"):
            sides = 4
        elif(shape1=="circle"):
            sides = 7


        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) == sides:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

            elif len(approx) > sides:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt 

        cv2.drawContours(img, [largest],0, boundary , 25)
        M = cv2.moments(largest)
        cx1 = int(M['m10']/M['m00'])
        cy1 = int(M['m01']/M['m00'])
        
    else:
        cx1 = cy1 = "None"

    ############## for Object2 ##############

    if not color2 == "none":
        if(color2=="red"):
            lower = np.array(red1)
            upper = np.array(red2)
            boundary = [0,255,0]

        elif(color2=="blue"):
            lower = np.array(blue1)
            upper = np.array(blue2)
            boundary = [0,0,255]

        elif(color2=="green"):
            lower = np.array(green1)
            upper = np.array(green2)
            boundary = [255,0,0]

        if(shape2=="triangle"):
            sides = 3
        elif(shape2=="square"):
            sides = 4
        elif(shape2=="circle"):
            sides = 7


        mask  = cv2.inRange(img, lower, upper)
        res   = cv2.bitwise_and(img, img, mask= mask)


        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)


        ret,thresh = cv2.threshold(gray,10,255,0)


        _,contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        
            if len(approx) == sides:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

            elif len(approx) > sides:
                if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
                    largest = cnt

        cv2.drawContours(img, [largest],0, boundary, 25)

        M = cv2.moments(largest)
        cx2 = int(M['m10']/M['m00'])
        cy2 = int(M['m01']/M['m00'])

    else:
        cx2 = cy2 = "None"

    cv2.imshow('CustomImage',img)
    cv2.waitKey(0)

    fileName = '../Images/'+img_name

    cv2.imwrite(fileName,img)

    xy1 = "("+str(cx1)+":"+str(cy1)+")"
    xy2 = "("+str(cx2)+":"+str(cy2)+")"
    
    global idno
    x = [str(aruco_name),idno,str(xy1),str(xy2)]

    with open('../2121_Task1.2.csv', 'a') as file:
         writer = csv.writer(file, delimiter = ',',quoting=csv.QUOTE_ALL)
         writer.writerow(x) 
              
        
    
    
                
                

if __name__ == "__main__": 

    # ---- Check if Images folder Exists. If not, create one. ----
    path = "../Images"
    if not os.path.exists(path):
        os.makedirs(path)

    with open('../2121_Task1.2.csv', 'w') as file:
            writer = csv.writer(file, delimiter = ',',quoting=csv.QUOTE_ALL)
            writer.writerow(['Image Name','Aruco ID','(x,y) Object-1',' (x,y) Object-2'])
    
    

    img_name = 'Image1.jpg'
    aruco_detect("../3. Images/Image1.jpg")

    img_name = 'Image2.jpg'
    aruco_detect("../3. Images/Image2.jpg")

    img_name = 'Image3.jpg'
    aruco_detect("../3. Images/Image3.jpg")

    img_name = 'Image4.jpg'
    aruco_detect("../3. Images/Image4.jpg")

    img_name = 'Image5.jpg'
    aruco_detect("../3. Images/Image5.jpg")
    

    print("\n***** The Images have been generated as given in Table 1, Table 2 and Table 3 of Task1.2.pdf *****")
    print("\n***** The Generalized code is now running *****\n")
    print("\nNOTE: Please put the input image in '3. Images' folder before proceeding!\n")

    choice = input("\nPress 0 to EXIT or ANY KEY to CONTINUE : ")

    while choice != '0':
        img_name = str(input("Enter file name: "))
        path = "../3. Images/"+img_name

        if not os.path.isfile(path):
            print("File does not exist!")
        else:
            
            n = input("Enter n (bits/'ARUCO'): ")
            C = input("Enter C (combination/'ORIGINAL') : ")
            flag = True

            if n == "ARUCO":
                if C == "ORIGINAL":
                    flag = True
                else:
                    flag=False
                    print("\nException : Invalid value of C (comnination).")

            elif not n.isdecimal() or (int(n)>7) or (int(n)<4):
                print("\nException : Invalid value of n (bits/'ARUCO').")
                flag = False

            elif not C.isdecimal() or ((int(C) != 50) and (int(C) != 100) and (int(C) != 250) and (int(C) != 1000)):
                print("\nException : Invalid value of C (comnination).")
                flag = False

            if flag:
                color1 = str(input("Enter Object 1 color [red,green,blue,none]: "))
                shape1 = str(input("Enter Object 1 shape [triangle,square,circle,none]: "))

                color2 = str(input("Enter Object 2 color [red,green,blue,none]: "))
                shape2 = str(input("Enter Object 2 shape [triangle,square,circle,none]: "))

                if(color1=="none" and not(shape1=="none") or (not color1=="none" and shape1=="none")):
                    print("\nException : Invalid Values for Object 1")

                elif(color2=="none" and not(shape2=="none") or (not color2=="none" and shape2=="none")):
                    print("\nException : Invalid Values for Object 2")

                elif ((not color1=="red" and not color1=="green" and not color1=="blue" and not color1=="none") or (not color2=="red" and not color2=="green" and not color2=="blue" and not color2=="none")) or ((not shape1=="triangle" and not shape1=="square" and not shape1=="circle" and not shape1=="none") or (not shape2=="triangle" and not shape2=="square" and not shape2=="circle" and not shape2=="none")) :
                    print("\nException : Invalid Value Entered")
                else:
                    custom_aruco_detect(path,n,C,color1,shape1,color2,shape2)


        choice = input("\nPress 0 to EXIT or ANY KEY to CONTINUE : ")

    