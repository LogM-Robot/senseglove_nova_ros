#!/usr/bin/env sh

# CTRL_DEVICE="E0:0A:F6:AA:2B:E2"
CTRL_DEVICE="F0:D4:15:3D:D5:EB" # LogM WS-101 bt mac
# CTRL_DEVICE="5C:87:9C:84:E5:67" # Laptop T580 bt mac
# CTRL_DEVICE="A4:B1:C1:C5:84:AA" # LogM WS-100 bt mac
SG_DEVICE="0C:DC:7E:9C:EC:26"
SG_RFCOMM="/dev/rfcomm0"

# 0C:DC:7E:9C:EC:26

bluetoothctl pairable on
bluetoothctl discoverable on
bluetoothctl pair ${SG_DEVICE}
bluetoothctl trust ${SG_DEVICE}
bluetoothctl connect ${SG_DEVICE}
sudo rfcomm connect ${SG_RFCOMM} ${SG_DEVICE} 1 &
