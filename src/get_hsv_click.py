import cv2 as cv
import numpy as np
 
 
def mouseRGB(event,x,y,flags,param):
    if event == cv.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = img[y,x,0]
        colorsG = img[y,x,1]
        colorsR = img[y,x,2]
        colors = img[y,x]
        hsv_value= np.uint8([[[colorsB ,colorsG,colorsR ]]])
        hsv = cv.cvtColor(hsv_value,cv.COLOR_BGR2HSV)
        print ("HSV : " ,hsv)
        print("Red: ",colorsR)
        print("Green: ",colorsG)
        print("Blue: ",colorsB)
        print("BRG Format: ",colors)
        print("Coordinates of pixel: X: ",x,"Y: ",y)
 
# Read an image, a window and bind the function to window
img = cv.imread("/home/learnroboticswros/catkin_ws/src/opencv_tutorial/images/output/camera_image.png")

cv.namedWindow('mouseRGB')
cv.setMouseCallback('mouseRGB',mouseRGB)
 
#Do until esc pressed
while(1):
    cv.imshow('mouseRGB',img)
    if cv.waitKey(20) & 0xFF == 27:
        break
#if esc pressed, finish.
cv.destroyAllWindows()