#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorFilter(object):

    def __init__(self):
        self.image_sub = rospy.Subscriber("/rgbd_camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding ="bgr8")
        except CvBridgeError as e:
            print(e)
    
        # convert image from RGB in HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        min_red = np.array([0, 100, 70])
        max_red = np.array([180, 255, 255])

        mask_r = cv2.inRange(hsv, min_red, max_red)

        res_r = cv2.bitwise_and(cv_image, cv_image, mask = mask_r)

        cv2.imshow('Original', cv_image)
        cv2.imshow('Mask on color', mask_r)
        cv2.imshow('Red', res_r)

        cv2.waitKey(1)





if __name__ == '__main__':
    
    color_filter_object = ColorFilter() 
    rospy.init_node('color_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()