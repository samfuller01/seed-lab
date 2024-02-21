#The program uses the webcam to detect a aruco marker. If there is an aruco marker then it detects what quadrant 
#the marker is in. It sends the quadrant the aruco marker is in to the arduino. Additionally it prints to the 
#LCD screen the position the wheels should be in, and prints to the command line the quadrant the marker is in. 
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus
import threading
import queue

#initialize lcd
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c,lcd_columns,lcd_rows)
lcd.clear()
lcd.backlight = False
lcd.color = [100,0,0]
lcd.text_direction = lcd.LEFT_TO_RIGHT
sleep(1)

#setting aruco library
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# initialize the camera. Channel 1 or 0. 
camera = cv2.VideoCapture(0)
# creates queue
q = queue.Queue()

#The function createString takes in an integer denoting the position of the marker, 
#and outputs a string of the goal position of the wheels, as well as printing out
#the quadrant the marker is in. 
def createString(number):
    string = "Goal Pos: "
    if number == 0:
        string = string + "0 0"
        print("NE")
    elif number == 1:
        string = string + "0 1"
        print("NW")
    elif number == 2:
        string = string + "1 1"
        print("SW")
    elif number == 3:
        string = string + "1 0"
        print("SE")

    return string

#disp function prints a string to the LCD from the queue. Then clears the queue. 
def disp():
    while True:
        if not q.empty():
            gotSomething = q.get()
            q.queue.clear()
            string = createString(gotSomething)
            lcd.message = string
    
# Let the camera warmup
sleep(0.1)

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 11
# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)
offset = 1;
command = 0;
#stores quadrant markers is in
#0-NE 1-NW 2-SW 3-SE

#Sets up thread. 
myThread = threading.Thread(target=disp,args=())
myThread.start()
#command is where quadrant information is stored 
command = -1
while(True):
    #camera reads in the image
    ret, image = camera.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert image to grayscale
    k = cv2.waitKey(1) & 0xFF
    #if q pressed exit loop
    if k == ord('q'):
        break
        
    #creates array of XY coordinates for each corner of the marker, an array of the 
    #markers id and an array of each rejected canidate center of image. 
    corners,ids,rejected = aruco.detectMarkers(gray,aruco_dict)

    #this takes the image dimensions and stores them in variables called height and width
    height, width, channels = image.shape
    #these apply two gray lines to the grayscale image, putting these lines to distinguish quadrants
    image = cv2.line(gray, (0,int(height/2)), (width, int(height/2)),(255,255,255),9)
    image = cv2.line(gray, (int(width/2),0), (int(width/2),height),(255,255,255),9)

    #this shows the image
    cv2.imshow('Image',gray)

    #if an aruco marker is detected
    if not ids is None:
        #this acquires the actual corner values returned from the tuple corner
        markerCorners = corners[0][0]

        #this sets the sum of all x components and y components of the corners into a single sum to be averaged later
        centerX = markerCorners[0][0] + markerCorners[1][0] + markerCorners[2][0] + markerCorners[3][0]
        centerY = markerCorners[0][1] + markerCorners[1][1] + markerCorners[2][1] + markerCorners[3][1]

        #this sets the center of the aruco marker by averaging the x and y components
        centerX /= 4
        centerY /= 4

        #this stores a variable meaning absolute center of x or y, which align with the lines drawn to distinguish quadrants
        absCenX = width/2
        absCenY = height/2

        #this creates booleans based on whether the aruco marker's center was left or above the crossing lines to identify which quadrant the aruco marker is in
        xLeft = centerX < absCenX
        yUp = centerY < absCenY

        
        temp = command #stores previous position
        #reads the position of the marker and stores it in command. 

        #this if statement sets command to 1 if the aruco marker is in the upper left quadrant
        if xLeft and yUp:
            command = 1
        #this elif statement sets command to 0 if the aruco marker is in the upper right quadrant
        elif (not xLeft) and yUp:
            command = 0
        #this elif statement sets command to 3 if the aruco marker is in the bottom right quadrant
        elif (not xLeft) and (not yUp):
            command = 3
        #this elif statement sets command to 2 if the aruco marker is in the bottom left quadrant
        elif xLeft and (not yUp):
            command = 2

        #if the quadrant is changed then send the new quadrant info to the Arduino and update the queue
        if command != temp:    
            i2c.write_byte_data(ARD_ADDR,offset,command)
            q.put(command)

#closing the program 
camera.release()
cv2.destroyAllWindows()
lcd.color = [0,0,0]
lcd.clear()


    

