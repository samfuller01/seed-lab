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
command = 0;#stores quadrant markers is in
#0-NE 1-NW 2-SW 3-SE
myThread = threading.Thread(target=disp,args=())
myThread.start()
command = -1
while(True):
    ret, image = camera.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert image to grayscale
    k = cv2.waitKey(1) & 0xFF
    #if key pressed exit loop
    if k == ord('q'):
        break
    #creates array of XY coordinates for each corner of marker, an array of the 
    #markers id and an array of each rejected canidate center of image. 
    corners,ids,rejected = aruco.detectMarkers(gray,aruco_dict)
    #if aruco detected print out id else print none found.
    height, width, channels = image.shape
    image = cv2.line(gray, (0,int(height/2)), (width, int(height/2)),(255,255,255),9)
    image = cv2.line(gray, (int(width/2),0), (int(width/2),height),(255,255,255),9)
    cv2.imshow('Image',gray)
    if not ids is None:
        markerCorners = corners[0][0]
        centerX = markerCorners[0][0] + markerCorners[1][0] + markerCorners[2][0] + markerCorners[3][0]
        centerY = markerCorners[0][1] + markerCorners[1][1] + markerCorners[2][1] + markerCorners[3][1]

        centerX /= 4
        centerY /= 4

        absCenX = width/2
        absCenY = height/2

        xLeft = centerX < absCenX
        yUp = centerY < absCenY

       
        temp = command #stores previous command
        if xLeft and yUp:
            command = 1
        elif (not xLeft) and yUp:
            command = 0
        elif (not xLeft) and (not yUp):
            command = 3
        elif xLeft and (not yUp):
            command = 2

    
        if command != temp:    
            i2c.write_byte_data(ARD_ADDR,offset,command)
            q.put(command)
        
camera.release()
cv2.destroyAllWindows()
lcd.color = [0,0,0]
lcd.clear()


    

