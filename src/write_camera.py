#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class WriteImage(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/rgbd_camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        folder_path = '/home/learnroboticswros/catkin_ws/src/opencv_tutorial/images/output/'  
        img = cv2.imwrite(folder_path + 'object_new_position.jpg', cv_image)
        cv2.imshow('image',cv_image)
        cv2.waitKey(0)



def main():
    write_image_object = WriteImage()
    rospy.init_node('write_image_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()