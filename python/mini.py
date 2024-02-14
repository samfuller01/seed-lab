from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus
import threading
import queue
from random import random

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

def createString(number):
    string = "Goal Position: "
    if number == 0:
        string = string + "0 0"
    elif number == 1:
        string = string + "0 1"
    elif number == 2:
        string = string + "1 1"
    elif number == 3:
        string = string + "1 0"
    q.put(string)
    
def display():
    if not q.empty():
        disp = str(q.get())
        lcd.write = disp
        
myThread = threading.Thread(target=display,args=())
myThread.start()
# Let the camera warmup
sleep(0.1)

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 11
# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)
offset = 1;
command = 0;#stores quadrant markers is in
#0-NE 1-NW 2-SW 3-SE
while(True):
    ret, image = camera.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert image to grayscale
    cv2.imshow('Image',gray)
    k = cv2.waitKey(1) & 0xFF
    #if key pressed exit loop
    if k == ord('q'):
        break
    #creates array of XY coordinates for each corner of marker, an array of the 
    #markers id and an array of each rejected canidate center of image. 
    corners,ids,rejected = aruco.detectMarkers(gray,aruco_dict)
    #if aruco detected print out id else print none found. 
    if not ids is None:
        command = 3 #reads quadrant of markers in
        i2c.write_byte_data(ARD_ADDR,offset,command)
        createString(command)
        
camera.release()
cv2.destroyAllWindows()
lcd.color = [0,0,0]
lcd.clear()


    

