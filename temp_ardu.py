#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 温度と気圧から高度を計算する
# 2022.4.12
# Windows 用
#

from cv2 import detail_BestOf2NearestRangeMatcher
import serial
import time
from datetime import datetime
import csv
import cv2
import os
import shutil
from dronekit import connect    # フライトコントローラやシミュレータへ接続するのがdronekit内にあるconnect
from pymavlink import mavutil

# ドローンに接続
# ここは各自の環境によって書き換える
# connection_string = "COM5,57600"
# 実機ドローンだとconnection_stringは"/dev/ttyUSB0,57600"の様になる
connection_string = "COM7,57600"
print( u"FCへ接続: %s" % (connection_string) )    # 接続設定文字列を表示
vehicle = connect(connection_string, wait_ready=False)    # 接続
print(vehicle.mode)

#定数と初期化
P = 1006
T = 24.3
P0 = 1013.25 # ICAO標準大気 海抜0m
cnt = 50
interval_time = 1
filepath = "data/temp/"
H0 = 0
DH0 = 0

def s16(value):
    return -(value & 0x8000) | (value & 0x7fff)

def calc_crc(buf, length):
    """
    CRC-16 calculation.
    """
    crc = 0xFFFF
    for i in range(length):
        crc = crc ^ buf[i]
        for i in range(8):
            carrayFlag = crc & 1
            crc = crc >> 1
            if (carrayFlag == 1):
                crc = crc ^ 0xA001
    crcH = crc >> 8
    crcL = crc & 0x00FF
    return (bytearray([crcL, crcH]))

def print_latest_data(data,csv_file):
    global H0
    global DH0
    """
    print measured latest value.
    """
    time_measured = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
    temperature = str( s16(int(hex(data[9]) + '{:02x}'.format(data[8], 'x'), 16)) / 100)
    relative_humidity = str(int(hex(data[11]) + '{:02x}'.format(data[10], 'x'), 16) / 100)
    ambient_light = str(int(hex(data[13]) + '{:02x}'.format(data[12], 'x'), 16))
    barometric_pressure = str(int(hex(data[17]) + '{:02x}'.format(data[16], 'x')
            + '{:02x}'.format(data[15], 'x') + '{:02x}'.format(data[14], 'x'), 16) / 1000)
    sound_noise = str(int(hex(data[19]) + '{:02x}'.format(data[18], 'x'), 16) / 100)
    eTVOC = str(int(hex(data[21]) + '{:02x}'.format(data[20], 'x'), 16))
    eCO2 = str(int(hex(data[23]) + '{:02x}'.format(data[22], 'x'), 16))
    discomfort_index = str(int(hex(data[25]) + '{:02x}'.format(data[24], 'x'), 16) / 100)
    heat_stroke = str(s16(int(hex(data[27]) + '{:02x}'.format(data[26], 'x'), 16)) / 100)
    vibration_information = str(int(hex(data[28]), 16))
    si_value = str(int(hex(data[30]) + '{:02x}'.format(data[29], 'x'), 16) / 10)
    pga = str(int(hex(data[32]) + '{:02x}'.format(data[31], 'x'), 16) / 10)
    seismic_intensity = str(int(hex(data[34]) + '{:02x}'.format(data[33], 'x'), 16) / 1000)
    DH = vehicle.location.global_relative_frame.alt

    P = float(barometric_pressure)
    T = float(temperature)
    # H = 44330.8 * ( 1 - ( P / P0 ) ** 0.190263 ) # **:べき乗
    H = ((P0/P) ** (1/5.257) - 1) * ( T + 273.15) / 0.0065
    # H = 153.8 * ( T + 273.2) * (1-( P / P0 ) ** 0.190223)
    high = str(H)
    drone_alt = str(DH)

    # 高度の差分を計算
    if H0 == 0:
        H0 = H 
        calc_high =  str(H - H0 )
    else:
        calc_high =  str(H - H0)
        H0 = H
    if DH0 == 0:
        DH0 = DH 
        D_calc_high =  str(DH - DH0 )
    else:
        D_calc_high =  str(DH - DH0)
        DH0 = DH

    print("")
    print("Time measured:" + time_measured)
    print("Temperature:" + temperature)
    print("Relative humidity:" + relative_humidity)
    print("Ambient light:" + ambient_light)
    print("Barometric pressure:" + barometric_pressure)
    print("Sound noise:" + sound_noise)
    print("eTVOC:" + eTVOC)
    print("eCO2:" + eCO2)
    print("Discomfort index:" + discomfort_index)
    print("Heat stroke:" + heat_stroke)
    print("Vibration information:" + vibration_information)
    print("SI value:" + si_value)
    print("PGA:" + pga)
    print("Seismic intensity:" + seismic_intensity)
    print("Altitude:" + high)
    print("Diff Altitude:" + calc_high)
    print("Drone Altitude:" + drone_alt)
    print("Diff Drone Altitude:" + D_calc_high)

    # Output CSV
    with open(csv_file, 'a', newline="") as f:
        writer = csv.writer(f)
        writer.writerow([time_measured, temperature, relative_humidity, ambient_light,
                         barometric_pressure, sound_noise, eTVOC, eCO2, discomfort_index,
                         heat_stroke, vibration_information, si_value, pga, seismic_intensity,high,calc_high,drone_alt,D_calc_high])

def worker(ser,csv_file):
    # Get Latest data Long.
    command = bytearray([0x52, 0x42, 0x05, 0x00, 0x01, 0x21, 0x50])
    command = command + calc_crc(command, len(command))
    tmp = ser.write(command)
    time.sleep(0.1)
    data = ser.read(ser.inWaiting())
    print_latest_data(data,csv_file)

#パラメータ　待機秒数、接続先、出力ファイル名
def scheduler(csv_file):
    # Serial.
    # linux対応
    # ser = serial.Serial("/dev/ttyUSB0", 115200, serial.EIGHTBITS, serial.PARITY_NONE)
    ser = serial.Serial("com6", 115200, serial.EIGHTBITS, serial.PARITY_NONE)
    worker(ser,csv_file)

def save_frame_camera_key(device_num, dir_path, basename,CSV_FILE, ext='jpg', window_name='frame'):
    cap = cv2.VideoCapture(device_num)

    if not cap.isOpened():
        return

    os.makedirs(dir_path, exist_ok=True)
    base_path = os.path.join(dir_path, basename)
    time_measured = datetime.now().strftime("%Y%m%d-%H%M%S")

    n = 0
    while True:
        ret, frame = cap.read()
        time_measured = datetime.now().strftime("%Y%m%d-%H%M%S")
        cv2.putText(frame, time_measured, (10, 30),cv2.FONT_HERSHEY_PLAIN, 1.5,(255, 255, 255), 1, cv2.LINE_AA)
        # cv2.imshow(window_name, frame)
        cv2.imwrite('{}_{}_{}.{}'.format(base_path,time_measured, n, ext), frame)

        #パラメータ　待機秒数、接続先、出力ファイル名
        scheduler(CSV_FILE)
        time.sleep(interval_time)

        n += 1
        if n > cnt:
            break
    cv2.destroyWindow(window_name)

if __name__ == '__main__':

    # 既存ファイルの削除
    try:
        shutil.rmtree(filepath)
    except:
        print("既存ファイル削除エラー")
    if os.path.exists(filepath):
        print("TEMPフォルダあり")
    else:
        os.mkdir(filepath) 
    
    # 出力するCSVファイル名
    str_time = str(datetime.now().strftime("%Y%m%d-%H%M%S"))
    CSV_FILE = filepath + str_time + ".csv"
    with open(CSV_FILE, 'w', newline="") as f:
        writer = csv.writer(f)
        writer.writerow(['Time measured', 'Temperature', 'Relative humidity', 'Ambient light',
                         'Barometric pressure', 'Sound noise', 'eTVOC', 'eCO2', 'Discomfort index',
                         'Heat stroke', 'Vibration information', 'SI value', 'PGA', 'Seismic intensity','Altitude','Diff_alt','Drone_alt','Drone_alt_diff'])

    save_frame_camera_key(0, filepath, 'camera_capture',CSV_FILE)
    print(str_time)
    shutil.make_archive('arc_data',format='zip',root_dir=filepath)
