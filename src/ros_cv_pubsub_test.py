#! /usr/bin/env python
# -- coding: utf-8 --
from time import sleep
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

def callback(msg):
    try:
            np_arr = np.fromstring(msg.data, np.uint8)
            input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    except Exception as err:
            rospy.loginfo('err')

    output_image = process_image(input_image)
    bridge = CvBridge()
    pub = rospy.Publisher('image_gray', Image, queue_size=10)
    pub.publish(bridge.cv2_to_imgmsg(output_image, "mono8"))

    node_name = "cv_bridge_demo_compressed"
    cv2.imshow(node_name, output_image)   
    cv2.waitKey(1)

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    rospy.Subscriber("/usb_cam1/image_raw/compressed", CompressedImage, callback)
    rospy.spin()

def process_image(frame):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grey = cv2.blur(grey, (7, 7))
    edges = cv2.Canny(grey, 15.0, 30.0)
    return edges

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
