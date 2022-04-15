#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# カメラキャプチャー
# 2022.4.15
# 

from datetime import datetime
import time
import cv2
import os

def save_frame_camera_key(device_num, dir_path, basename, ext='jpg', delay=1, window_name='frame'):
    cap = cv2.VideoCapture(device_num)

    if not cap.isOpened():
        return

    os.makedirs(dir_path, exist_ok=True)
    base_path = os.path.join(dir_path, basename)

    n = 0
    while True:
        ret, frame = cap.read()
        # cv2.imshow(window_name, frame)
        cv2.imwrite('{}_{}.{}'.format(base_path, n, ext), frame)
        n += 1
        if n > 10:
            break
        time.sleep(1)
    cv2.destroyWindow(window_name)

time_measured = str(datetime.now().strftime("%Y%m%d-%H%M%S"))
# save_frame_camera_key(0, 'data/temp', 'camera_capture')
print(time_measured)
save_frame_camera_key(0, 'data/temp', time_measured)
