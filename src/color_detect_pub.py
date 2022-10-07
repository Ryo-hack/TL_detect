#! /usr/bin/env python
# -- coding: utf-8 --
from time import sleep
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class color_detect :
    def __init__(self):
        self.node_name = "color_detect"
        self.bridge = CvBridge()
        rospy.init_node(self.node_name)
        self.image_pub = rospy.Publisher('output_image', Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/usb_cam1/image_raw/compressed", CompressedImage, self.image_callback)

    def image_callback(self,image_msg):
        try:
                np_arr = np.fromstring(image_msg.data, np.uint8)
                input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as err:
                rospy.loginfo('err')

        output_image = self.process_image(input_image)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "8UC1"))




    def process_image(self, frame):
        
        ### ここに加工処理などを記述する ###
        # フレーム待ち
        img = frame
        #RGB    
        red_R_min = 83
        red_R_max = 255
        red_G_min = 0
        red_G_max = 50
        red_B_min = 0
        red_B_max = 100

        R_min = 0
        R_max = 0
        G_min = 50
        G_max = 255 
        B_min = 30
        B_max = 255

        bgrLower = np.array([B_min, G_min, R_min])    # 抽出する色の下限(BGR)
        bgrUpper = np.array([B_max, G_max, R_max])    # 抽出する色の上限(BGR)
        img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
        
        red_bgrLower = np.array([red_B_min, red_G_min, red_R_min])    # 抽出する色の下限(BGR)
        red_bgrUpper = np.array([red_B_max, red_G_max, red_R_max]) 
        red_img_mask = cv2.inRange(img, red_bgrLower, red_bgrUpper)


        mask_1 = cv2.inRange(img, (0, 127, 127), (30, 255, 255))
        mask_2 = cv2.inRange(img, (0, 0, 0), (255, 255, 255))
        img_ = cv2.bitwise_or(mask_1, mask_2)

        #論理演算で色検出
        dst = cv2.bitwise_and(img_, img_, mask=img_mask)
        red_dst = cv2.bitwise_and(img_, img_, mask=red_img_mask)

        # 表示
        images = np.hstack((dst))
        cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color_image', img)
        cv2.namedWindow('detect_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('detect_image', dst)
        cv2.namedWindow('red_detect_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('red_detect_image', red_dst)
        cv2.waitKey(1)
        return dst


if __name__ == '__main__':
    color_detect()
    rospy.spin()  


