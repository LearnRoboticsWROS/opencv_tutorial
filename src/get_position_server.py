#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from position_tracker.srv import GetPosition, GetPositionResponse

class ObjectDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.server = rospy.Service("/get_position", GetPosition, self.handle_get_position)
    
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
    
            
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        min_red = np.array([0,20,160])
        max_red = np.array([5, 120, 226])

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
        
        #print("how many object I detect: ", len(object_detected))
        #print(object_detected)
                
        x_pxl_center = 319
        y_pxl_center = 240
        pxl_mm_conversion = 27/20
        x0 = 1000
        y0 = 0
        z0 = 500

        for cnt in object_detected:
            rect = cv.minAreaRect(cnt)
            (x_center, y_center), (w,h), orientation = rect
            #print("width = ", w)
            #print("heigh = ", h)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.polylines(cv_image, [box], True, (255, 0,0),1)

            # at the center x=1, y = 0, z = 0.5

            #condition top right of the imgage
            if (x_center > x_pxl_center) and (y_center < y_pxl_center):
                y = y0 - (x_center-x_pxl_center)/pxl_mm_conversion
                x = x0+(y_pxl_center - y_center)/pxl_mm_conversion
                z = z0
            
            #condition bottom right of the imgage
            elif (x_center > x_pxl_center) and (y_center > y_pxl_center):
                y = y0 - (x_center-x_pxl_center)/pxl_mm_conversion  
                x = x0 - (y_center - y_pxl_center)/pxl_mm_conversion  
                z = z0

            #condition bottom left of the imgage
            elif (x_center < x_pxl_center) and (y_center > y_pxl_center): 
                y = y0 + (x_pxl_center-x_center)/pxl_mm_conversion  
                x = x0 - (y_center - y_pxl_center)/pxl_mm_conversion 
                z = z0

            #condition top left of the imgage
            elif (x_center < x_pxl_center) and (y_center < y_pxl_center):
                y = y0 + (x_pxl_center-x_center)/pxl_mm_conversion
                x = x0 + (y_pxl_center - y_center)/pxl_mm_conversion
                z = z0

            elif (x_center == x_pxl_center) and (y_center == y_pxl_center):
                x= x0
                y= y0
                z = z0

            self.x_pos = x
            self.y_pos = y
            self.z_pos = z

            cv.putText(cv_image, "x: {}".format(round(x, 1)) + " y: {}".format(round(y,1)), (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0),1)
            cv.circle(cv_image, (int(x_center), int(y_center)), 1, (255,0,0), thickness=-1)
            #cv.putText(cv_image, "x: {}".format(round(x_center, 1)) + " y: {}".format(round(y_center,1)), (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (255,0,0),1)
            #cv.circle(cv_image, (int(x_center), int(y_center)), 1, (255,0,0), thickness=-1)
        #cv.imshow("cropped", cropped_img)
        cv.imshow("obj detection from red mask world coordinate", cv_image)
        cv.waitKey(1)

    def handle_get_position(self, req):
        if req:
            res = GetPositionResponse()
            res.x_position = self.x_pos
            res.y_position = self.y_pos
            res.z_position = self.z_pos

            rospy.loginfo("Object position: x = %f, y = %f, z = %f" %(self.x_pos, self.y_pos, self.z_pos))
        return res

if __name__ == '__main__':
    
    object_detection = ObjectDetection() 
    rospy.init_node('object_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()