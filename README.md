# D1-Arm-Control-UPDATED
Controls for the unitree D1 arm. as of 10/02/2026, we can move joints as we please, and then use forward kinematics to find the position and rotation of the end effector.

To get the Arm set up for every reconnect, do the following commands.

sudo ip addr add 192.168.123.10/24 dev enp0s31f6
sudo ip link set enp0s31f6 up
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
cd ~/unitree_d1_teleoperation/d1_sdk/build

To do any movement with the arm, always zero it out. "./arm_zero_control"

"./multiple_joint_angle_control" allows you to adjust each joint angle as you want. move to whichever joint you want, and adjust to your desired angle. 

to test out the forward kinematics, once the arm is in a desired position, run "./get_arm_pose".
