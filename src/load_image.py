#!/usr/bin/env python3

import rospy
import cv2

class LoadImage:
    def __init__(self, picture):
        self.picture = picture
    
    def read_photo(self):
        img = cv2.imread(self.picture)
        cv2.imshow('image', img)
        cv2.waitKey(0)

if __name__ == '__main__':
    image = '/home/learnroboticswros/catkin_ws/src/opencv_tutorial/images/input/6-axis-duo.png'
    load_image_object = LoadImage(image)
    load_image_object.read_photo()
