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
        cv2.imshow('detect_image', dst)
        cv2.imshow('red_detect_image', red_dst)
        sleep(0.01) 
    ### ここに加工処理などを記述する ###

        if cv2.waitKey(1) & 0xff == 27:#ESCで終了
            writer.release()
            video.release()
            cv2.destroyAllWindows()
            break
  


print("---end---")



