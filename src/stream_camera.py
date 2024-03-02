#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObjectDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rgbd_camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
    
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # crop the image
        #cropped_img = cv_image[307:410, 290:520]
        

        # convert the image to grayscale
        #gray = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
        #cv.imshow("gray", gray)

        #mask = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 3, 3)
        #cv.imshow("mask", mask)
            
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        min_red = np.array([0, 100, 70])
        max_red = np.array([180, 255, 255])

        mask_r = cv.inRange(hsv, min_red, max_red)
        mask = cv.adaptiveThreshold(mask_r, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 3, 3)
        cv.imshow("mask", mask)

        # find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        #print("contours: ", contours)

        for cnt in contours:
            cv.polylines(cv_image, [cnt], True, [255, 0, 0], 1)

        object_detected = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 20:
                cnt = cv.approxPolyDP(cnt, 0.03*cv.arcLength(cnt, True), True)
                object_detected.append(cnt)
        
        print("how many object I detect: ", len(object_detected))
        #print(object_detected)

        for cnt in object_detected:
            rect = cv.minAreaRect(cnt)
            (x_center, y_center), (w,h), orientation = rect
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.polylines(cv_image, [box], True, (255, 0,0),1)
            cv.putText(cv_image, "x: {}".format(round(x_center, 1)) + " y: {}".format(round(y_center,1)), (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (255,0,0),1)
            cv.circle(cv_image, (int(x_center), int(y_center)), 1, (255,0,0), thickness=-1)



        #cv.imshow("cropped", cropped_img)
        cv.imshow("obj detection from red mask", cv_image)
        cv.waitKey(1)




if __name__ == '__main__':
    
    object_detection = ObjectDetection() 
    rospy.init_node('object_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()