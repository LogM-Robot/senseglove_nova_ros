#!/usr/bin/env sh
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

bluetoothctl disconnect ${SG_DEVICE}
rfcomm release ${SG_RFCOMM}
