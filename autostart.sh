#!/bin/sh

python3 /home/pi/hello.py
python3 /home/pi/.local/bin/mavproxy.py --master=/dev/serial0 --baudrate 921600 --out=udpbcast:192.168.0.13:14550


