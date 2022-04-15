#!/bin/sh
echo "Hello World"

sudo modprobe ftdi_sio
sudo chmod 777 /sys/bus/usb-serial/drivers/ftdi_sio/new_id
python3 temp1.py


exit 0

