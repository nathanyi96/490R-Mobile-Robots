rosbag record -b 1 -o test.bag /vesc/low_level/ackermann_cmd_mux/input/teleop /initialpose
rosbag play --hz 20 /vesc/low_level/ackermann_cmd_mux/input/teleop
