# -*- coding: utf-8 -*-
"""
**************************************************************************
*                  E-Yantra Robotics Competition
*                  ================================
*  This software is intended to check version compatiability of open source software
*  Theme: ANT BOT
*  MODULE: Task1.1
*  Filename: Task1.1.py
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
"""

import numpy
import cv2
import cv2.aruco as aruco
import os
from pathlib import Path

mappingName = {}


def aruco_gen(id_aruco, num_pixels):

    # Setting up n and C according to the id_aruco                               #replace n with no. of bits per pixel and C with the no. of combinations
                                                           		#select n and C from the list mentioned above

    if id_aruco == 8 or id_aruco == 27:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    elif id_aruco == 92 or id_aruco == 4:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)



    # ----------Building the Marker----------
   	                                                           
    img = aruco.drawMarker(aruco_dict, id_aruco,num_pixels)


    tempImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    finalImg = cv2.copyMakeBorder(tempImg,25,25,25,25,cv2.BORDER_CONSTANT,value = (255,255,255))


    # -------------- Adding Red label to RGB File ------------

    title = 'ArUco ID = '+ str(id_aruco)

    fontStyle = cv2.FONT_HERSHEY_SIMPLEX

    cv2.putText(finalImg, title , (170, 17), fontStyle, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

    # ------------- Creating the .jpg file from RGB file -----------

    fileName = '../Images/ArUco'+ str(id_aruco) +'.jpg'

    tempName = 'Aruco'+str(id_aruco)+'.jpg'
    global mappingName
    mappingName[tempName] = 1

    cv2.imwrite(fileName,finalImg)

    # ------------ Display RGB Image -----

    cv2.imshow('frame',finalImg)


    cv2.waitKey(0)

    cv2.destroyAllWindows()


def generalised_version(id_aruco, n, C, num_pixels):

    global c

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
    
    # ----------Building the Marker----------
   	                                                           
    img = aruco.drawMarker(aruco_dict, id_aruco,num_pixels)

    tempImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    finalImg = cv2.copyMakeBorder(tempImg,25,25,25,25,cv2.BORDER_CONSTANT,value = (255,255,255))

    # -------------- Adding Red label to RGB File ------------

    title = 'ArUco ID = '+ str(id_aruco)

    fontStyle = cv2.FONT_HERSHEY_SIMPLEX

    cv2.putText(finalImg, title , (170, 17), fontStyle, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

    # ------------- Creating the .jpg file from RGB file -----------

    fileName = '../Images/ArUco'+ str(id_aruco) +'.jpg'

    global mappingName
    tempName = 'Aruco'+str(id_aruco)+'.jpg'
    
    if tempName in mappingName:
        fileName = '../Images/Aruco'+str(id_aruco)+'_'+str(mappingName[tempName])+'.jpg'
        print('\n"' +tempName+'"','already present. New name is : '+'"'+ 'Aruco'+str(id_aruco) + '_' + str(mappingName[tempName]) + '.jpg' + '"')
        mappingName[tempName] +=1

    else:
        mappingName[tempName] = 1

    cv2.imwrite(fileName,finalImg)

    # ------------ Display RGB Image -----

    cv2.imshow('frame',finalImg)

    cv2.waitKey(0)

    cv2.destroyAllWindows()
    


if __name__ == "__main__":   
    c = 3
    aruco_gen(8, 400)
    aruco_gen(27, 400)
    aruco_gen(92, 400)
    aruco_gen(4, 400)

    print("\n\n*** THE 4 ARUCO MARKERS HAVE BEEN GENERATED AS SPECIFIED IN TABLE 1 of Task1.1.pdf ***\n")

    print("*** Now the Generalised Code is running ***\n")

    choice = input("\nPress 0 to EXIT or ANY KEY to CONTINUE : ")

    while choice != '0':

        id_aruco = input("\nEnter ArUco ID : ")
        n = input("Enter n (bits/'ARUCO'): ")
        C = input("Enter C (combination/'ORIGINAL') : ")

        if (not id_aruco.isdecimal()) or (int(id_aruco) > 1023):
            print("Exception : Invalid value of ArUco ID.")

        elif(n == 'ARUCO'):
            if(C == 'ORIGINAL'):
                generalised_version(int(id_aruco),n,C,400)
            else:
                print("Exception : Invalid Value of C (combination/'ORIGINAL').")

        elif not n.isdecimal() or (int(n)>7) or (int(n)<4):
            print("Exception : Invalid value of n (bits/'ARUCO').")

        elif not C.isdecimal() or ((int(C) != 50) and (int(C) != 100) and (int(C) != 250) and (int(C) != 1000)):
            print("Exception : Invalid value of C (comnination).")

        
        elif int(id_aruco) >= int(C):
            print("Exception : Aruco ID exceeding C (combination).")

        elif(int(id_aruco) < int(C)):
            generalised_version(int(id_aruco),n,C,400)
            
        else:
            print("Exception : Invalid Dictionary.")
        

        #------ Choose to Exit program or not.------

        choice = input("\nPress 0 to EXIT or ANY KEY to CONTINUE : ")
    
    
