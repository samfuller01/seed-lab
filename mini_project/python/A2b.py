'''
The following performs the actions outlined by Assignment 2 part 2b.

    -Purpose: The following code takes a picture, applies a mask with
    reductions applied to the mask, and then detects a green shape.

'''


#Import the necessary libraries
from time import sleep
import numpy as np
import cv2
from smbus2 import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(1)

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

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8
# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)

# Let the camera warmup
sleep(0.1)
# Get an image from the camera stream
ret, image = camera.read()

#Turns the camera off and lets it power down
camera.release() 
sleep(0.1)


#Applies the open morthology to the image, which is an erosion, then dilation
#A second erosion was applied but commented out due to poorer quality
kernel = np.ones((5,5),np.uint8)
image1 = cv2.morphologyEx(image,cv2.MORPH_OPEN,kernel)


#Shows the image with the applied morphology
cv2.imshow("Morphology",image)



#This line converts the image to HSV values from BGR values
imageHSV = cv2.cvtColor(image1,cv2.COLOR_BGR2HSV)


#These are the bounds found for the upper and lower thresholds for green
lowerGreen = np.array([35, 70, 1])
upperGreen = np.array([78, 255, 255])

#This creates the mask with the thresholds 
mask = cv2.inRange(imageHSV,lowerGreen,upperGreen)

#This shows the mask
cv2.imshow("mask",mask)




#These apply contours to the image and applies it to the image
#This is based on the mask
contourGreenVis = image.copy()

contoursGreen,_ = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(contourGreenVis,contoursGreen,-1,(255,0,0),3)


#This creates the image for denoting which shapes in the photo
#are green. A green box with the text "Green Shape" will appear
greenBox = image.copy()
greenCenters = np.empty((0,2))
greenAreas = np.empty((0))


#This loop will mark anything that has a contourArea of greater than 300
#Then it will draw the green box with the text
for idx, cnt in enumerate(contoursGreen):
    contourArea = cv2.contourArea(cnt)
    if contourArea > 300:
        x, y, w, h = cv2.boundingRect(cnt)
        center = int(x+w/2),int(y+h/2)
        greenAreas = np.append(greenAreas,contourArea)
        greenCenters = np.vstack((greenCenters,center))
        cv2.rectangle(greenBox, (x, y), (x + w, y + h), (0, 255, 0), 1)
        cv2.putText(greenBox,'Green Shape',(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,0),1)
        cv2.putText(greenBox, '+', center, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)

#This shows the marked image and then closes all windows based upon a key press
cv2.imshow("Detection",greenBox)    
cv2.waitKey(0)
cv2.destroyAllWindows()

