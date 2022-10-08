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
        self.TL_param = rospy.search_param('TL_param_config.yml')
        self.node_name = "color_detect"
        self.topic_name = "/image_raw"
        self.bridge = CvBridge()
        rospy.init_node(self.node_name)
        self.image_pub = rospy.Publisher('output_image', Image, queue_size=10)
        self.state_pub = rospy.Publisher("tl_state", String, queue_size=10)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)

        self.blue_threshold = rospy.get_param('~blue_threshold',100)     
        self.blue_h_mim  = rospy.get_param('~blue_h_mim',80)
        self.blue_s_mim = rospy.get_param('~blue_s_mim',80)
        self.blue_v_mim = rospy.get_param('~blue_v_mim',0)
        self.blue_h_max  = rospy.get_param('~blue_h_max',115)
        self.blue_s_max = rospy.get_param('~blue_s_max',127)
        self.blue_v_max = rospy.get_param('~blue_v_max',200)

        self.red_threshold = rospy.get_param('~red_threshold',100)
        self.red_h_mim = rospy.get_param('~red_h_mim',0)
        self.red_s_mim = rospy.get_param('~red_s_mim',0)
        self.red_v_mim = rospy.get_param('~red_v_mim',76)
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
        #色検出
        blue_img_mask = cv2.inRange(img, np.array([self.blue_h_mim, self.blue_s_mim,self.blue_v_mim]), np.array([self.blue_h_max , self.blue_s_max , self.blue_v_max ]))
        red_img_mask = cv2.inRange(img, np.array([self.red_h_mim,self.red_s_mim,self.red_v_mim]), np.array([self.red_h_max,self.red_s_max, self.red_v_max]))
        
        #色面積
        redPixels = cv2.countNonZero(red_img_mask)
        bluePixels = cv2.countNonZero(blue_img_mask)

        #表示用に青と赤の結果を合体
        red = (red_img_mask==255)
        blue = (blue_img_mask==255)
        img_mask = deepcopy(img)
        img_mask[red] = [0,0,255]
        img_mask[blue] = [255,0,0]

        # state publisher
        state_msg=String()
        if redPixels >=self.blue_threshold:
            state_msg.data = "red"
        elif bluePixels >=self.red_threshold:
            state_msg.data="blue"
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


