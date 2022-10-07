#! /usr/bin/env python
# -- coding: utf-8 --
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String

class color_detect :
    def __init__(self):
        self.node_name = "color_detect"
        self.topic_name = "/usb_cam1/image_raw/compressed"
        self.bridge = CvBridge()
        rospy.init_node(self.node_name)
        self.image_pub = rospy.Publisher('output_image', Image, queue_size=10)
        self.stop_pub = rospy.Publisher("stop", String, queue_size=10)
        self.go_pub = rospy.Publisher("go", String, queue_size=10)
        self.image_sub = rospy.Subscriber(self.topic_name, CompressedImage, self.image_callback)
        #self.blue_threshold 
        #self.blue_h 
        #self.blue_s
        #self.blue_v
        #self.red_threshold
        #self.red_h
        #self.red_s
        #self.red_v


    def image_callback(self,image_msg):
        try:
                np_arr = np.fromstring(image_msg.data, np.uint8)
                input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as err:
                rospy.loginfo('err')

        output_image = self.process_image(input_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "8UC1"))


    def process_image(self, frame):### ここに加工処理などを記述する ###
        # フレーム待ち
        img = frame
        #色検出
        blue_img_mask = cv2.inRange(img, np.array([80, 80,0]), np.array([115, 127, 200]))
        red_img_mask = cv2.inRange(img, np.array([0, 0,76]), np.array([35, 127, 200]))
        #色面積
        redPixels = cv2.countNonZero(red_img_mask)
        bluePixels = cv2.countNonZero(blue_img_mask)
        if redPixels >=100:
            rospy.loginfo('stop')
            msg = "stop {}".format(rospy.get_time())
            self.stop_pub.publish(msg)
        if bluePixels >=100:
            msg = "go {}".format(rospy.get_time())
            self.go_pub.publish(msg)
            rospy.loginfo('go')
        # 表示
        cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color_image', img)
        cv2.namedWindow('blue_detect_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('blue_detect_image', blue_img_mask)
        cv2.namedWindow('red_detect_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('red_detect_image', red_img_mask)
        cv2.waitKey(1)

        return  red_img_mask


if __name__ == '__main__':
    color_detect()
    rospy.spin()  


