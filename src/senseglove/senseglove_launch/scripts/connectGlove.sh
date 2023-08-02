#!/bin/bash
# if arg $2 is true, then launch sgc_l.sh, else launch sgc_r.sh
if [ $2 == true ]
then
    sudo bash $6/scripts/sgc_l.sh &
fi
if [ $3 == true ]
then
    sudo bash $6/scripts/sgc_r.sh &
fi

sleep 5

# Start SenseCom
chmod +x $1/SenseCom/Linux/SenseCom.x86_64
$1/SenseCom/Linux/SenseCom.x86_64 &

# Allow SenseComm to start
sleep 5

# Start SenseGlove node
source ~/Documents/LfD_tactile/Avatar-ROS/devel/setup.bash
roslaunch senseglove_launch senseglove_hardware_demo.launch left:=$2 right:=$3 tracker_L_serial:=$4 tracker_R_serial:=$5


# Kill SenseComm when node is killed
killall -9 SenseCom.x86_64
if [ $2 == true ]
then
    sudo bash $6/scripts/sgd_l.sh
fi
if [ $3 == true ]
then
    sudo bash $6/scripts/sgd_r.sh
fi
