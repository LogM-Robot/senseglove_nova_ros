#!/usr/bin/env sh

SG_DEVICE="0C:DC:7E:9C:E6:8A"
SG_RFCOMM="/dev/rfcomm0"

bluetoothctl disconnect ${SG_DEVICE}
rfcomm release ${SG_RFCOMM}
