#The program uses the webcam to detect a aruco marker. If there is an aruco marker then it detects the angle 
#the marker is from the center. It then displays this angle on the LCD screen. Each time the angles changes
#the program will update the LCD screen with the new angle. After detecting a marker we send the angle to the ardiuno. 
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
import board

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
hf = 51.12615099
#fudge factor
hf=hf+1.3
#half field of view
half = .5*hf
#sets up thread
myThread = threading.Thread(target=disp,args=())
myThread.start()
#initialize angle
angle = 0
reply = 0

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 11
# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)
offset = 0
#variable we send to Arduino
command = 0
#placeholder variable
peep = 0
while(True):
    #camera reads in the image
    ret, image = camera.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert image to grayscale
    k = cv2.waitKey(1) & 0xFF
    #if q pressed exit loop
    #if k == ord('q'):
        #break
        
    #creates array of XY coordinates for each corner of the marker, an array of the 
    #markers id and an array of each rejected canidate center of image. 
    corners,ids,rejected = aruco.detectMarkers(gray,aruco_dict)

    #this takes the image dimensions and stores them in variables called height and width
    height, width, channels = image.shape
    #applies a vertical line to center of camera
    image = cv2.line(gray, (int(width/2),0), (int(width/2),height),(255,255,255),2)

    #if an aruco marker is detected
    if not ids is None:
        #tells arduino the marker has been detected
        if reply == 0:
            command = 50
            i2c.write_byte_data(ARD_ADDR,offset,command)
            reply = i2c.read_byte_data(ARD_ADDR,offset)
            reply = 1
            sleep(0.1)
            
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
        #stores previous angle
        temp = angle
        #calculation for angle
        angle = -half*(centerX-320)/(320)
        #if angle changed update lcd
        if angle != temp:
            q.put(round(angle,4))

        #sends angle across once
        if reply == 1:
            command = round(angle)
            command = int(command)
            i2c.write_byte_data(ARD_ADDR,offset,command)
            reply = 2
            peep = 1
            break
    #tells arduino the camera is on      
    if peep == 0:
        command = 100
        i2c.write_byte_data(ARD_ADDR,offset,command)
        command = 0
        peep = 1
            

        
#closing the program
camera.release()
cv2.destroyAllWindows()
lcd.color = [0,0,0]
lcd.clear()


    

