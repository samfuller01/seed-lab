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
import glob
import math

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

#disp function prints a string to the LCD from the queue. Then clears the queue. 
def disp():
    while True:
        if not q.empty():
            gotSomething = q.get()
            q.queue.clear()
            string = str(gotSomething)
            lcd.message = string
    
# Let the camera warmup
sleep(0.1)
#frame of view of the camera
fov = 68.5
hf = 57.154313995636249434982928784921329730700701401395403860031
#sets up thread
myThread = threading.Thread(target=disp,args=())
myThread.start()
angle = 0 
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
    #image = cv2.line(gray, (0,int(height/2)), (width, int(height/2)),(255,255,255),9)
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
        
        temp = angle
        angle = -1*.5*hf*(centerX-width/2)/(width/2)
        if angle != temp:
            q.put(round(angle,4))
	
    

        
#closing the program 
camera.release()
cv2.destroyAllWindows()
lcd.color = [0,0,0]
lcd.clear()


    

