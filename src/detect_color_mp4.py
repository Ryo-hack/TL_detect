#! /usr/bin/env python
# -- coding: utf-8 --
from time import sleep
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist

###################
print("---start---")
#動画ファイルを読み込む
video = cv2.VideoCapture("./test.mp4")

# 幅と高さを取得
width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
size = (width, height)

#総フレーム数を取得
frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

#フレームレート(1フレームの時間単位はミリ秒)の取得
frame_rate = int(video.get(cv2.CAP_PROP_FPS))


for i in range(frame_count):
        ret, frames = video.read()
### ここに加工処理などを記述する ###
        # フレーム待ち

        img = frames

        blue_img_mask =  cv2.inRange(img, np.array([80, 80,0]), np.array([115, 127, 200]))
        red_img_mask = cv2.inRange(img, np.array([0, 0,76]), np.array([35, 127, 200]))

        # 表示
        cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color_image', img)
        cv2.imshow('blue_detect_image', blue_img_mask)
        cv2.imshow('red_detect_image',red_img_mask)
        sleep(0.01) 
    ### ここに加工処理などを記述する ###

        if cv2.waitKey(1) & 0xff == 27:#ESCで終了
            writer.release()
            video.release()
            cv2.destroyAllWindows()
            break
  


print("---end---")



