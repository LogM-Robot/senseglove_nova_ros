#!/usr/bin/env sh

# CTRL_DEVICE="E0:0A:F6:AA:2B:E2"
CTRL_DEVICE="F0:D4:15:3D:D5:EB" # LogM WS-101 bt mac
SG_SERIAL_L="0C:DC:7E:9C:E6:8A"
SG_SERIAL_R="0C:DC:7E:9C:EC:26"
if [ "$1" = "false" ]; then
    SG_DEVICE=${SG_SERIAL_L}
elif [ "$1" = "true" ]; then
    SG_DEVICE=${SG_SERIAL_R}
else
    echo "Invalid argument: $1"
    exit 1
fi
# SG_DEVICE="0C:DC:7E:9C:E6:8A"
SG_RFCOMM="/dev/rfcomm0"

# 0C:DC:7E:9C:EC:26

bluetoothctl pairable on
bluetoothctl discoverable on
bluetoothctl pair ${SG_DEVICE}
bluetoothctl trust ${SG_DEVICE}
bluetoothctl connect ${SG_DEVICE}
rfcomm connect ${SG_RFCOMM} ${SG_DEVICE} 1 &
