# Minnie
Assembly instructions and software for a minimalistic, economical robot.

---

After following setup instructions (to be provided), execute each of the following instructions in its own terinal tab...

roscore
rosrun rosserial_python serial_node.py /dev/ttyS0 _baud:=115200
roslaunch minnie_control minnie_hardware.launch
roslaunch minnie_control minnie_visualize.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

(At some point, the above will be replaced by roslaunch minnie_bringup)

---

I am providing code in this repository to you under an open source license. Because this is my personal repository, the license you receive to my code is from me and not from my employer (Facebook).
