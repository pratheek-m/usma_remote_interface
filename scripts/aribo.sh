# Realtex NIC, wlan2, to ARIBOwifi
# ARIBOwifi password is aribopass

# PICe, wlan0, to GVR-BOT-004
# gvr_bot password is modern0325

# IP address of Apach server is 192.168.1.101
# IP adress of GVR_Bot is 192.168.0.101

# Unplug camera and plug back in if it does discover it

source ~/.bashrc

sudo chmod a+rw /dev/ttyUSB0

sudo route add 192.168.17.214 eth0

roslaunch aribo master.launch

cd ~/catkin_ws/src/gvr_bot_bridge/resources
rosrun gvr_bot_bridge gvr_bot_bridge_node 




