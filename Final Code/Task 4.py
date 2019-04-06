'''
 * Team Id : 2121
 * Author List : Tamsil Sajid Amani, Mohd Bilal Aziz, Mohammed Omar Siddiqui, Zaid Hasan
 * Filename: eYRC#AB#2121.py
 * Theme: Ant Bot (AB)
 * Functions: capture_image, detect_Marker, findID, straight, reverse, soft_turn, reverse_soft_turn, get_sensor_values, move_straight,
                move_reverse, aruco_left_turn, bfs, find_shortest_distance, find_qah, get_supply_location, find_closest_unvisited_shrub, scan_image, start_servicing,
                sharp_move_left, sharp_move_right, command_robot
 * Global Variables: Motor1A, Motor1B, Motor1E, Motor2A, Motor2B, Motor2E, rightMotor, leftMotor, rightThreshold, midThreshold, leftThreshold, max_right, max_left, dutyCycleLeft,
                dutyCycleRight, list_of_id, serialInput, up, down, left, right, direction, red, green, blue, red_color_code, green_color_code, blue_color_code, yellow_color_code,
                robotHeading, robotLocation, number_of_nodes, graph, serviceNodeDictionary, shrub_nodes, shrub_visited, trash_node, lastCommandReverse, pid
'''
import heapq
import sys
import time
from multiprocessing import Process
from time import sleep

import RPi.GPIO as GPIO
import cv2
import cv2.aruco as aruco
import numpy as np
import picamera
import serial
from picamera import PiCamera
from picamera.array import PiRGBArray

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

Motor1A = 33  # Motor1A:
Motor1B = 35
Motor1E = 31  # changed
Motor2A = 36
Motor2B = 38
Motor2E = 40

GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)
GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)
GPIO.setup(Motor2E, GPIO.OUT)

rightMotor = GPIO.PWM(Motor1E, 1000)  # rightMotor: assigns right DC motor to 1000 PWM
leftMotor = GPIO.PWM(Motor2E, 1000)  # leftMotor: assigns left DC motor to 1000 PWM

rightThreshold = 200  # rightThreshold: Threshold value of the right white line sensor below which it is white and above which the detected surface is taken as black
midThreshold = 200  # midThreshold: Threshold value of the middle white line sensor below which it is white and above which the detected surface is taken as black
leftThreshold = 200  # leftThreshold: Threshold value of the left white line sensor below which it is white and above which the detected surface is taken as black

max_right = 550  # max_right : approx. maximum value given by the right photo-receptor of White Line Sensor
max_left = 550  # max_left: approx. maximum value given by the left photo-receptor of White Line Sensor
min_right = 30  # min_right : approx. minimum value given by the right photo-receptor of White Line Sensor
min_left = 30  # min_left : approx. minimum value given by the left photo-receptor of White Line Sensor

Kp = 35  # pid: amount of variation assigned to dutyCycleRight and dutyCycleLeft

dutyCycleLeft = 40  # dutyCycleLeft: assigns duty cycle to left DC motor
dutyCycleRight = 40  # dutyCycleRight: assigns duty cycle to right DC motor

lastNodeDetectedTime = time.clock()

list_of_id = []  # list_of_id: contains the ArUco ID's of the scanned ArUco markers

serialInput = serial.Serial("/dev/ttyUSB0", 9600)  # serialInput: Used to take input from arduino

up = 'U'  # up: contains direction of robot i.e. UP
down = 'D'  # up: contains direction of robot i.e. DOWN
left = 'L'  # up: contains direction of robot i.e. LEFT
right = 'R'  # up: contains direction of robot i.e. RIGHT

direction = [up, right, down, left]  # direction: order in which the robot has to take turn i.e. either right or left.

red = 'red'
green = 'green'
blue = 'blue'
red_color_code = 1  # red_color_code: assigns 1 to red color
green_color_code = 2  # green_color_code: assigns 2 to green color
blue_color_code = 3  # blue_color_code: assigns 3 to blue color
yellow_color_code = 4  # yellow_color_code: assigns 4 to yellow color

robotHeading = right  # robotHeading: indicates the direction of robot
robotLocation = 7  # robotLocation: tells us at which node robot is currently

number_of_nodes = 31  # number_of_nodes: total number of nodes on the arena
graph = [[' ' for c in range(number_of_nodes)] for r in
         range(number_of_nodes)]  # graph: contains the paths between the nodes

# Shrub area
graph[0][7] = up
graph[1][8] = up
graph[2][9] = up
graph[3][10] = up
graph[4][11] = up
graph[5][12] = up
graph[6][13] = up

graph[7][0] = down
graph[8][1] = down
graph[9][2] = down
graph[10][3] = down
graph[11][4] = down
graph[12][5] = down
graph[13][6] = down

graph[7][8] = right
graph[8][9] = right
graph[9][10] = right
graph[10][11] = right
graph[11][12] = right
graph[12][13] = right

graph[8][7] = left
graph[9][8] = left
graph[10][9] = left
graph[11][10] = left
graph[12][11] = left
graph[13][12] = left

# Center
graph[21][10] = down
graph[21][22] = right
graph[21][20] = left
graph[21][29] = up

graph[10][21] = up
graph[22][21] = left
graph[20][21] = right
graph[29][21] = down

graph[29][30] = up
graph[30][29] = down

# Ant hill 3
graph[14][15] = right
graph[15][16] = right
graph[15][20] = up

graph[15][14] = left
graph[16][15] = left
graph[20][15] = down

# Ant hill 2
graph[17][18] = right
graph[18][19] = right
graph[18][22] = up

graph[18][17] = left
graph[19][18] = left
graph[22][18] = down

# Ant hill 0
graph[23][24] = right
graph[24][25] = right
graph[24][20] = down

graph[24][23] = left
graph[25][24] = left
graph[20][24] = up

# Ant hill 1
graph[26][27] = right
graph[27][28] = right
graph[27][22] = down

graph[27][26] = left
graph[28][27] = left
graph[22][27] = up

serviceNodeDictionary = {
    # serviceNodeDictionary: assigns the node numbers of the services to their respective Ant-Hills
    0: [23, 25],
    1: [26, 28],
    2: [17, 19],
    3: [14, 16]
}

shrub_nodes = {  # shrub_nodes: contains the locations of the supplies
    red_color_code: [],
    green_color_code: [],
    blue_color_code: []
}

shrub_visited = {  # shrub_visited: tells whether a shrub has been visited or not
    0: False,
    1: False,
    2: False,
    4: False,
    5: False,
    6: False
}

box_nodes = [0, 1, 2, 4, 5, 6, 14, 16, 17, 19, 23, 25, 26,
             28]  # Contains list of nodes for which path last path traversal is not taken
trash_node = 30  # trashNode: node where we deposit the trash
lastCommandReverse = False  # lastCommandReverse: tells us whether the robot has taken a revere move

'''
*Function Name: clamp() 
*Input: value, min_value, max_value
*Output: Reutrn the duty cycle value
*Logic: Checks if the duty cycle exceeds the maximum i.e 100. If it is, then it returns the minimum of max value and max of min value and value
*Example Call: clamp(46, 10, 100)
'''


def clamp(value, min_value, max_value):
    return min(max_value, max(min_value, value))


'''
*Function Name: pick() 
*Input: None
*Output: Picks up the supply/trash blocks
*Logic: Sends "P" to arduino denoting that it has to pick the block
*Example Call: pick()
'''


def pick():
    serialInput.write("P".encode())
    sleep(4)


'''
*Function Name: place() 
*Input: None
*Output: Places down the supply/trash blocks
*Logic: Sends "Q" to arduino denoting that it has to place the block
*Example Call: place()
'''


def place():
    serialInput.write("Q".encode())
    sleep(4)


'''
*Function Name: capture_image 
*Input: name -> stores the name of the image file to be created
*Output: Captures an image from the Pi Camera 
*Logic: Open the Pi Camera and takes the picture and saves the file with 'name' parameter
*Example Call: capture_image('SIM0.jpg')
'''

count = 0


def capture_image():
    global count
    camera = PiCamera()
    camera.resolution = (320, 240)
    camera.framerate = 90
    rawCapture = PiRGBArray(camera, size=(320, 240))
    sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        image = frame.array
        # rawCapture.truncate()

        # cv2.imshow("frame", image)
        key = cv2.waitKey(1) & 0xFF
        detect_marker(image)
        rawCapture.truncate(0)
        if count is 4:
            break
        # if key == ord("q"):
        #      break


'''
*Function Name: detect_marker 
*Input: name -> stores the name of the image file captured from the Pi Camera
*Output: Writes the id of the AruCo marker detected to `eYRC#AB#2121.csv` file
*Logic: It calls the findID function to get the detected id of the AruCo marker
*Example Call: detect_marker('SIM0.jpg')
'''


def detect_marker(name):
    global count
    # img = cv2.imread(name)
    gray = cv2.cvtColor(name, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()

    aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
    markerid = findID(gray, aruco_dict, parameters=parameters)
    if markerid is not 0:
        print("ID DETECTED:", markerid)
        count += 1
        return


'''
*Function Name: findID 
*Input: gray -> Gray scale file containing the AruCo marker,
        aruco_dict -> Contains the specific dictionary against which we check the marker for id,
        parameters -> Contains default parameter values which are to be used in detectMarkers function
*Output: Returns the id of the marker detected
*Logic: Makes use of the cv2.aruco library to detect the marker id
*Example Call: findID(gray, aruco_dict,parameters=parameters)
'''


def findID(gray, aruco_dict, parameters):
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if corners and ids[0][0] not in list_of_id:
        list_of_id.append(ids[0][0])
        return ids[0][0]
    return 0


'''
*Function Name: straight 
*Input: None
*Output: Moves the line follower in a straight line
*Logic: Both the motors are rotated in the same direction to move the line follower in the forward direction
*Example Call: straight()
'''


def straight():
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    leftMotor.start(dutyCycleLeft)

    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    rightMotor.start(dutyCycleRight)


'''
*Function Name: reverse 
*Input: None
*Output: Moves the line follower in a straight line but in reverse direction
*Logic: Both the motors are rotated in the same direction to move the line follower in the backward direction
*Example Call: reverse()
'''


def reverse():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    leftMotor.start(dutyCycleLeft)

    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    rightMotor.start(dutyCycleRight)


'''
*Function Name: soft_turn
*Input: temp_cycle_left -> value used for duty cycle of left DC motor, 
        temp_cycle_right -> value used for duty cycle of right DC motor,
        take_reverse -> rotates the dc motor clock-wise or anti-clockwise 
*Output: Turn the line follower in the left/right direction to keep it on a straight line
*Logic: The right and left motors are rotated with different speeds to change direction
*Example Call: soft_turn(50, 40)
'''


def soft_turn(temp_cycle_right, temp_cycle_left, take_reverse=False):
    temp_cycle_left = clamp(temp_cycle_left, 10, 100)
    temp_cycle_right = clamp(temp_cycle_right, 10, 100)
    if not take_reverse:
        GPIO.output(Motor1A, GPIO.HIGH)
        GPIO.output(Motor1B, GPIO.LOW)
        GPIO.output(Motor2A, GPIO.HIGH)
        GPIO.output(Motor2B, GPIO.LOW)
    else:
        GPIO.output(Motor1A, GPIO.LOW)
        GPIO.output(Motor1B, GPIO.HIGH)
        GPIO.output(Motor2A, GPIO.LOW)
        GPIO.output(Motor2B, GPIO.HIGH)

    rightMotor.start(temp_cycle_right)
    leftMotor.start(temp_cycle_left)


'''
*Function Name: get_sensor_values 
*Input: None
*Output: Gives the readings of the three sensor values
*Logic: reads the output from arduino and removes the unnecessary characters
*Example Call: get_sensor_values()
'''


def get_sensor_values():
    t2 = str(serialInput.readline()).strip("b'").strip("\\n").strip("\\r")

    t1 = t2.split(' ')

    if len(t1) is not 3:
        return get_sensor_values()
    val1, val2, val3 = t1[0], t1[1], t1[2]
    if (not str(val1).isdigit()) or (not str(val2).isdigit()) or (not str(val3).isdigit()):
        return get_sensor_values()
    val1, val2, val3 = int(t1[0]), int(t1[1]), int(t1[2])
    return val1, val2, val3


'''
*Function Name: move_straight 
*Input: move_time -> time for which the bot will move straight(seconds)
*Output: Makes sure the robot moves in the black line
*Logic: Calculates the duty cycle dynamically using 'pid' and calls the straight, soft_turn functions as per obtained sensor values
*Example Call: move_straight()
'''


def move_straight(move_time=0.0):
    global dutyCycleRight
    global dutyCycleLeft

    right_value, mid_value, left_value = get_sensor_values()

    while (right_value > rightThreshold and mid_value > midThreshold) or (
            left_value > leftThreshold and mid_value > midThreshold):
        right_value, mid_value, left_value = get_sensor_values()

        temp_cycle_left = dutyCycleLeft + (((leftThreshold - left_value) * Kp) / (max_left - min_left))
        temp_cycle_right = dutyCycleRight + (((rightThreshold - right_value) * Kp) / (max_right - min_right))
        soft_turn(temp_cycle_left, temp_cycle_right)

    node_start_time = time.clock()
    while True:
        right_value, mid_value, left_value = get_sensor_values()
        global lastNodeDetectedTime
        time_diff = time.clock() - lastNodeDetectedTime

        if (right_value > rightThreshold and mid_value > midThreshold) or (
                left_value > leftThreshold and mid_value > midThreshold):
            lastNodeDetectedTime = time.clock()
            leftMotor.start(0)
            rightMotor.start(0)
            # print("node detected")
            # print(right_value, mid_value, left_value)
            # straight()
            # sleep(1)
            break

        else:
            if right_value < rightThreshold and mid_value > midThreshold and left_value < leftThreshold:
                straight()
                continue
            temp_cycle_left = dutyCycleLeft + (((leftThreshold - left_value) * Kp) / (max_left - min_left))
            temp_cycle_right = dutyCycleRight + (((rightThreshold - right_value) * Kp) / (max_right - min_right))
            soft_turn(temp_cycle_left, temp_cycle_right)



'''
*Function Name: move_reverse
*Input: None
*Output: Makes sure the robot moves on the black line in backward direction
*Logic: Calculates the duty cycle dynamically using 'pid' and calls the reverse, reverse_soft_turn functions as per obtained sensor values
*Example Call: move_reverse()
'''


def move_reverse():
    global dutyCycleRight
    global dutyCycleLeft

    right_value, mid_value, left_value = get_sensor_values()

    while (right_value > rightThreshold and mid_value > midThreshold) or (
            left_value > leftThreshold and mid_value > midThreshold):
        right_value, mid_value, left_value = get_sensor_values()

        temp_cycle_left = dutyCycleLeft + (((leftThreshold - left_value) * Kp) / (max_left - min_left))
        temp_cycle_right = dutyCycleRight + (((rightThreshold - right_value) * Kp) / (max_right - min_right))
        soft_turn(temp_cycle_left, temp_cycle_right, True)

    node_start_time = time.clock()
    while True:
        right_value, mid_value, left_value = get_sensor_values()
        global lastNodeDetectedTime
        if (right_value > rightThreshold and mid_value > midThreshold) or (
                left_value > leftThreshold and mid_value > midThreshold):
            lastNodeDetectedTime = time.clock()
            # print("time taken", time.clock() - node_start_time)
            leftMotor.start(0)
            # print("node detected")
            rightMotor.start(0)
            break

        else:
            if right_value < rightThreshold and midThreshold > 300 and left_value < leftThreshold:
                reverse()
                continue
            temp_cycle_left = dutyCycleLeft + (((leftThreshold - left_value) * Kp) / (max_left - min_left))
            temp_cycle_right = dutyCycleRight + (((rightThreshold - right_value) * Kp) / (max_right - min_right))
            soft_turn(temp_cycle_left, temp_cycle_right, True)


'''
*Function Name: aruco_left_turn(time) 
*Input: time -> Time in seconds to turn the robot by 45 and 90 degrees
*Output: Turn the line follower in the left direction with a delay to turn it by 45 and 90 degrees
*Logic: The right motor is rotated in forward direction and the left motor is rotated in backward direction to turn the robot left
*Example Call: turnLeft(0.7)
'''


def aruco_left_turn(time):
    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)

    rightMotor.start(dutyCycleRight)
    leftMotor.start(dutyCycleLeft)
    sleep(time)
    leftMotor.start(0)
    rightMotor.start(0)


'''
*Function Name: bfs
*Input: source -> starting node, 
        destination -> final node to reach, 
        predecessor -> conatins list of previusly visited nodes, 
        distance -> total number of edges between the source and destination
*Output: Makes sure the robot moves on the black line in backward direction
*Logic: Calculates the duty cycle dynamically using 'pid' and calls the reverse, reverse_soft_turn functions as per obtained sensor values
*Example Call: move_reverse()
'''


def bfs(source, destination, predecessor, distance):
    queue = []
    visited = [False for _ in range(number_of_nodes)]
    distance[source] = 0
    queue.append(source)
    visited[source] = True

    while len(queue) is not 0:
        current_node = queue.pop(0)

        for node in range(number_of_nodes):
            if graph[current_node][node] is not ' ' and visited[node] is False:
                visited[node] = True
                distance[node] = distance[current_node] + 1
                predecessor[node] = current_node
                queue.append(node)

                if node is destination:
                    return


'''
*Function Name: find_shortest_distance
*Input: source -> starting node of the robot, 
        destination -> final node to reach, 
*Output: path -> list containing the directions to reach the destination from source,
         distance -> Number of edges between the source and destination
*Logic: calls the 'bfs()' function to get the path and checks if destination can be visited 
*Example Call: find_shortest_distance(3,8)
'''


def find_shortest_distance(source, destination):
    distance = [-1 for _ in range(number_of_nodes)]
    predecessor = [-1 for _ in range(number_of_nodes)]
    bfs(source, destination, predecessor, distance)
    path = []
    crawl = destination

    while predecessor[crawl] is not -1:
        path.insert(0, graph[predecessor[crawl]][crawl])
        crawl = predecessor[crawl]
    return path, distance[destination]


'''
*Function Name: find_qah
*Input: ant_hill_info -> list containing binary representation of the 4 ArUco IDs 
*Output: Ant Hill number of Queen Ant Hill if present else -1
*Logic: checks the Most Significant Bit for '1'
*Example Call: find_qah(['00010010','00101010','00001111','11001100'])
'''


def find_qah(ant_hill_info):
    for x in range(len(ant_hill_info)):
        if ant_hill_info[x][0] is '1':
            return x

    return -1


'''
*Function Name: get_supply_location
*Input: reuqired_color_code -> contains the color code of the supply to be picked
*Output: return the node number of the required shrub which is closest from the current position of the robot 
*Logic: checks if closest shrub node is already there in 'shrub_nodes' and calculates the distance from the current node.
        Else it goes to the unvisited shrub nodes and scans for new supplies and checks the nearest distance available
*Example Call: get_supply_location(2)
'''


def get_supply_location(required_color_code):
    # If color box(s) was previously visited return the node closest to the current position
    if len(shrub_nodes[required_color_code]) is not 0:
        closest_node = None
        min_distance = sys.maxsize
        for node in shrub_nodes[required_color_code]:
            _, distance = find_shortest_distance(robotLocation, node)
            if distance < min_distance:
                min_distance = distance
                closest_node = node
        shrub_nodes[required_color_code].remove(closest_node)
        return closest_node


'''
*Function Name: find_closest_unvisited_shrub
*Input: None
*Output: return the node number of the closest required supply if both the locations of that supply are known. 
*Logic: uses 'find_shortest_distance' function of calculate the length of path till both the supplies and returns the node number of the closer one
*Example Call: find_closest_unvisited_shrub()
'''


def find_closest_unvisited_shrub():
    min_distance = sys.maxsize
    closest_node = None
    for shrub in shrub_visited:
        if shrub_visited[shrub] is False:
            _, distance = find_shortest_distance(robotLocation, shrub)
            if distance < min_distance:
                min_distance = distance
                closest_node = shrub
    return closest_node


'''
*Function Name: scan_image
*Input: None
*Output: returns the color code i.e 1 for Red, 2 for Green, 3 for Blue and 4 for Yellow. And also sends the arduino signal to light the LED with detected color of the supply
*Logic: uses the cv2 functions to identify the 4 colors and checks for the maximum area to know that a particular color is present
*Example Call: scan_image()
'''


def scan_image():
    # serialInput.write("A".encode())

    with picamera.PiCamera() as camera:
        camera.resolution = (1024, 768)
        camera.start_preview()
        sleep(1)
        camera.capture('temp_color.jpeg')

    image = cv2.imread("temp_color.jpeg")

    rgby_lower = [[15, 15, 122], [40, 56, 12], [105, 60, 2], [5, 98, 93]]  # Red,Green, Blue, Yellow
    rgby_upper = [[76, 75, 190], [75, 120, 75], [169, 80, 89], [60, 151, 156]]
    character = "A"  # "A" is for Red, "B" is for Green, "C" is for Blue, "D" is for Yellow

    for color in range(4):

        lower = np.array(rgby_lower[color])
        upper = np.array(rgby_upper[color])
        img = image

        mask = cv2.inRange(img, lower, upper)
        res = cv2.bitwise_and(img, img, mask=mask)

        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        # cv2.imshow('frame', gray)
        # cv2.waitKey(0)

        ret, thresh = cv2.threshold(gray, 50, 255, 0)

        _, contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = None
        for cnt in contours:

            if largest is None or cv2.contourArea(cnt) >= cv2.contourArea(largest):
                largest = cnt

        # if largest is not None:
        #     print("largest", cv2.contourArea(largest))

        if largest is not None and cv2.contourArea(largest) > 30000:
            color += 1
            serialInput.write(character.encode())
            return color + 1
        character = chr(ord(character) + 1)
    return -1


'''
*Function Name: start_servicing
*Input: ant_hill_no -> stores the number of the ant hill to be serviced now,
        service_one_value -> stores the requirement number of service 1. i.e 1 for HoneyDew, 2 for Leaves and 3 for Wood,
        service_two_value -> stores the requirement number of service 2. i.e 1 for HoneyDew, 2 for Leaves and 3 for Wood,
        trash_value -> stores 0 for no trash and 1 for trash
*Output: Completes the required servies of that ant hill.
*Logic: commands the robot to reach a particular destination i.e. supplies or trash, and deposit them at any service location or trash deposit zone
*Example Call: start_servicing(1, 2, 3, 0)
'''


def start_servicing(ant_hill_no, service_one_value, service_two_value, trash_value):
    service_one_node = serviceNodeDictionary[ant_hill_no][0]
    service_two_node = serviceNodeDictionary[ant_hill_no][1]

    if service_one_value is 0 and service_two_value is 0 and trash_value is 1:
        command_robot(service_one_node)
        if scan_image() is not yellow_color_code:
            command_robot(service_two_node)
        scan_image()
        pick()
        command_robot(trash_node)
        place()

    elif service_one_value is 0 and service_two_value is not 0:
        supply_location = get_supply_location(service_two_value)
        command_robot(supply_location)
        scan_image()
        pick()
        command_robot(service_two_node)
        place()

        if trash_value is 1:
            command_robot(service_one_node)
            scan_image()
            pick()
            command_robot(trash_node)
            place()

    elif service_one_value is not 0 and service_two_value is 0:
        supply_location = get_supply_location(service_one_value)
        command_robot(supply_location)
        pick()
        command_robot(service_one_node)
        place()

        if trash_value is 1:
            command_robot(service_two_node)
            scan_image()
            pick()
            command_robot(trash_node)
            place()

    if service_one_value is not 0 and service_two_value is not 0:
        supply_location = get_supply_location(service_one_value)
        command_robot(supply_location)
        scan_image()
        pick()
        command_robot(service_one_node)
        place()

        supply_location = get_supply_location(service_two_value)
        command_robot(supply_location)
        scan_image()
        pick()
        command_robot(service_two_node)
        place()


'''
*Function Name: sharp_move_right
*Input: time_len -> time for which the robot moves straight,
        n -> decides for what angle our robot should move
*Output: Turn the robot 90 degree to its right
*Logic: turn right DC motor anti-clockwise and left DC motor clockwise to take a right turn
*Example Call: sharp_move_right
'''


def sharp_move_right(time_len=0.3, n=1):
    straight()
    sleep(0.3)
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    rightMotor.start(30)
    leftMotor.start(30)
    sleep(time_len)
    # sleep(1.8)

    # right_value, mid_value, left_value = get_sensor_values()
    # while right_value > rightThreshold or mid_value > midThreshold or left_value > leftThreshold:
    #     right_value, mid_value, left_value = get_sensor_values()
    #     continue
    #
    # while mid_value < midThreshold and right_value < rightThreshold:
    #     right_value, mid_value, left_value = get_sensor_values()
    #     continue

    rightMotor.start(0)
    leftMotor.start(0)

    global robotHeading

    robotHeading = direction[(direction.index(robotHeading) + n + len(direction)) % len(direction)]


'''
*Function Name: sharp_move_left
*Input: time_len -> time for which the robot moves straight,
        n -> decides for what angle our robot should move
*Output: Turn the robot 90 degree to its left
*Logic: turn right DC motor clockwise and left DC motor anti-clockwise to take a left turn
*Example Call: sharp_move_left
'''


def sharp_move_left(time_len=2.46, n=1):
    straight()
    sleep(0.3)
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    rightMotor.start(30)
    leftMotor.start(30)
    sleep(time_len)

    # right_value, mid_value, left_value = get_sensor_values()
    # while right_value > rightThreshold or mid_value > midThreshold or left_value > leftThreshold:
    #     print("in loop 1")
    #     right_value, mid_value, left_value = get_sensor_values()
    #     continue

    # print(right_value, mid_value, left_value)
    # print("on white")

    # while mid_value < midThreshold:
    #     right_value, mid_value, left_value = get_sensor_values()
    #     # print("in loop 2")
    #     continue

    # print(right_value, mid_value, left_value)
    # print("on black line")

    rightMotor.start(0)
    leftMotor.start(0)

    global robotHeading

    robotHeading = direction[(direction.index(robotHeading) - n + len(direction)) % len(direction)]


'''
*Function Name: command_robot
*Input: destination -> contains the node number to which the robot has to reach
*Output: drives the robot to the reuqired destination
*Logic: calculates the path using 'find_shortest_path()' function. Then takes each edge(step) and compares with current direction and turn the robot right or left according to the
        requirement.
*Example Call: command_robot(28)
'''


def command_robot(destination):
    # print("in command robot")
    global robotLocation
    global robotHeading
    # global lastCommandReverse
    path, distance = find_shortest_distance(robotLocation, destination)

    # print("Path = ", path)

    for path_number in range(len(path)):
        path_direction = path[path_number]

        if ((robotHeading is up and path_direction is down) or (robotHeading is down and path_direction is up) or
                (robotHeading is left and path_direction is right) or (
                        robotHeading is right and path_direction is left)):
            sharp_move_right(0, 2)
            move_straight()

        elif ((robotHeading is up and path_direction is left) or (robotHeading is left and path_direction is down) or \
              (robotHeading is down and path_direction is right) or (robotHeading is right and path_direction is up)):
            # print('sharp left')
            sharp_move_left()
            if destination in box_nodes and path_number is len(path) - 1:
                move_straight(0.2)
                robotLocation = destination
                return
            move_straight()

        elif (robotHeading is up and path_direction is right) or (robotHeading is right and path_direction is down) or \
                (robotHeading is down and path_direction is left) or (robotHeading is left and path_direction is up):
            # print('sharp right')
            sharp_move_right()
            if destination in box_nodes and path_number is len(path) - 1:
                move_straight(0.2)
                robotLocation = destination
                return
            move_straight()

        else:
            # print('straight')

            move_straight()

    robotLocation = destination


def scan_and_store(start):
    color_code = scan_image()
    if color_code is 1:
        shrub_nodes[red_color_code].append(7)
    elif color_code is 2:
        shrub_nodes[green_color_code].append(7)
    elif color_code is 3:
        shrub_nodes[blue_color_code].append(7)
    sleep(3)

    for x in range(start, 14):
        if x is 10:
            command_robot(11)
            continue

        command_robot(x)
        straight()
        sleep(0.25)
        leftMotor.start(0)
        rightMotor.start(0)
        color_code = scan_image()
        sleep(3)

        if color_code is 1:
            shrub_nodes[red_color_code].append(x)
        elif color_code is 2:
            shrub_nodes[green_color_code].append(x)
        elif color_code is 3:
            shrub_nodes[blue_color_code].append(x)

    # print("shrub nodes", shrub_nodes)


if __name__ == "__main__":
    #Threading to run two processes simultaneously
    p1 = Process(target=capture_image)
    p1.start()
    p2 = Process(target=command_robot(29))
    p2.start()

    p2.join()

    sharp_move_left(2.45, 2)

    p2 = Process(target=command_robot(10))
    p2.start()

    p2.join()
    p1.join()

    command_robot(7)
    sharp_move_right(1.35)
    sharp_move_right(0, 1)

    # sharp_move_right(time_len=2.45, n= 2)

    scan_and_store(8)

    sharp_move_left(2.45, 2)
    command_robot(10)
    sharp_move_left(1.35, n=1)


    # # # ---- Capturing Done
    ID0 = list_of_id[2]
    ID1 = list_of_id[1]
    ID2 = list_of_id[0]
    ID3 = list_of_id[3]
    #
    ant_hill_info = ['{0:08b}'.format(ID0), '{0:08b}'.format(ID1), '{0:08b}'.format(ID2), '{0:08b}'.format(ID3)]
    #
    qah = int(find_qah(ant_hill_info))
    priority_list = []
    service_two = []
    service_one = []
    trash = []
    #
    NoServiceRank = 0
    trashRank = 1
    ServiceRank = 2
    #
    for x in range(0, 4):
        AntHillNo = int(ant_hill_info[x][1:3], 2)
        service_two.append(int(ant_hill_info[x][3:5], 2))
        service_one.append(int(ant_hill_info[x][5:7], 2))
        trash.append(int(ant_hill_info[x][7]))

        if x is qah:
            priority_list.insert(AntHillNo, -50)

        elif service_one[x] is 0 and service_two[x] is 0:
            priority_list.insert(AntHillNo, (NoServiceRank + NoServiceRank + trash[x]))


        elif (service_one[x] is 0 and service_two[x] is not 0) or (service_one[x] is not 0 and service_two[x] is 0):
            priority_list.insert(AntHillNo, (NoServiceRank + ServiceRank + trash[x]))


        elif (service_one[x] is not 0 and service_two[x] is not 0) and trash[x] is 0:
            priority_list.insert(AntHillNo, (ServiceRank + ServiceRank))

    temp_list = priority_list.copy()
    heapq.heapify(temp_list)

    AntHillServiced = [False, False, False, False]
    #
    for x in range(0, 4):
        maxPriority = heapq.heappop(temp_list)
        AntHillNo = priority_list.index(maxPriority)
        priority_list[AntHillNo] = 100

        start_servicing(AntHillNo, service_one[AntHillNo], service_two[AntHillNo], trash[AntHillNo])

    command_robot(3)
    serialInput.write("Z".encode())
    sleep(10)
