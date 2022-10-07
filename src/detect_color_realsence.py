#! /usr/bin/env python
# -- coding: utf-8 --

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist

# ストリーム(Color/Depth)の設定
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# ストリーミング開始
pipeline = rs.pipeline()
profile = pipeline.start(config)

cv2.namedWindow("OpenCV Window")


    # トラックバーのコールバック関数は何もしない空の関数
def nothing(x):
    pass

 

try:
    while True:
        # フレーム待ち
        frames = pipeline.wait_for_frames()

        #RGB
        RGB_frame = frames.get_color_frame()
        RGB_image = np.asanyarray(RGB_frame.get_data())

        img = RGB_image

        #RGB値
        R_min = 83
        R_max = 255
        G_min = 0
        G_max = 50
        B_min = 0
        B_max = 100

        bgrLower = np.array([B_min, G_min, R_min])    # 抽出する色の下限(BGR)
        bgrUpper = np.array([B_max, G_max, R_max])    # 抽出する色の上限(BGR)
        img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
        
        mask_red1 = cv2.inRange(img, (0, 127, 127), (30, 255, 255))
        mask_red2 = cv2.inRange(img, (240, 127, 127), (255, 255, 255))
        img = cv2.bitwise_or(mask_red1, mask_red2)

        #論理演算で色検出
        dst = cv2.bitwise_and(img, img, mask=img_mask)
       

        # 表示
        images = np.hstack((dst))
        cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color_image', dst)
        if cv2.waitKey(1) & 0xff == 27:#ESCで終了
            cv2.destroyAllWindows()
            break

finally:
   # ストリーミング停止
    pipeline.stop()
