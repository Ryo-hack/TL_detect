#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from ast import Str
from email.mime import image
import numpy as np
import cv2
import rospy
from copy import deepcopy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String

class color_detect :
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("color_detect")
        self.image_pub = rospy.Publisher('output_image', Image, queue_size=10)
        self.state_pub = rospy.Publisher("tl_state", String, queue_size=10)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)

        self.blue_threshold = rospy.get_param('~blue_threshold',0.1)     
        self.blue_h_min  = rospy.get_param('~blue_h_min',80)
        self.blue_s_min = rospy.get_param('~blue_s_min',50)
        self.blue_v_min = rospy.get_param('~blue_v_min',0)
        self.blue_h_max = rospy.get_param('~blue_h_max',115)
        self.blue_s_max = rospy.get_param('~blue_s_max',127)
        self.blue_v_max = rospy.get_param('~blue_v_max',170)

        self.red_threshold = rospy.get_param('~red_threshold',0.1)
        self.red_h_min = rospy.get_param('~red_h_min',0)
        self.red_s_min = rospy.get_param('~red_s_min',0)
        self.red_v_min = rospy.get_param('~red_v_min',76)
        self.red_h_max = rospy.get_param('~red_h_max',35)
        self.red_s_max = rospy.get_param('~red_s_max',127)
        self.red_v_max = rospy.get_param('~red_v_max',200)

    def image_callback(self,image_msg):

        #try:
        #        np_arr = np.fromstring(image_msg.data, np.uint8)
        #        input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #except Exception as err:
        #        rospy.loginfo('err')
        input_image = self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
        output_image = self.process_image(input_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "bgr8"))
        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_msg))


    def process_image(self, frame):### ここに加工処理などを記述する ###
        # フレーム待ち
        img = frame
        height, width, channels = img.shape[:3]
        #色検出
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blue_img_mask = cv2.inRange(hsv, np.array([self.blue_h_min, self.blue_s_min,self.blue_v_min]), np.array([self.blue_h_max , self.blue_s_max , self.blue_v_max ]))
        red_img_mask = cv2.inRange(hsv, np.array([self.red_h_min,self.red_s_min,self.red_v_min]), np.array([self.red_h_max,self.red_s_max, self.red_v_max]))
        
        #色面積
        redPixels = cv2.countNonZero(red_img_mask)
        bluePixels = cv2.countNonZero(blue_img_mask)

        kernel = np.ones((3,3),np.uint8)
        blue_img_mask = cv2.erode(blue_img_mask,kernel,iterations = 1)
        #blue_img_mask = cv2.morphologyEx(blue_img_mask, cv2.MORPH_CLOSE, kernel)

        #表示用に青と赤の結果を合体
        red = (red_img_mask==255)
        blue = (blue_img_mask==255)
        img_mask = deepcopy(img)
        img_mask[red] = [0,0,255]
        img_mask[blue] = [255,0,0]

        # state publisher
        state_msg=String()
        if redPixels >=self.red_threshold*(height*width):
            state_msg.data = "red" #+str(redPixels/(height*width))
        elif bluePixels >=self.blue_threshold*(height*width):
            state_msg.data="blue" #+str(bluePixels/(height*width))
        else:
            state_msg.data = "unknown"
        self.state_pub.publish(state_msg)

        #print(redPixels)
        # 表示
        #cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('color_image', img)
        #cv2.namedWindow('blue_detect_image', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('blue_detect_image', blue_img_mask)
        #cv2.namedWindow('red_detect_image', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('red_detect_image', red_img_mask)
        #cv2.waitKey(1)

        return  img_mask


if __name__ == '__main__':
    color_detect()
    rospy.spin()  


