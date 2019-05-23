#!/bin/bash

rosparam set /pid/kp 1.0
rosparam set /pid/kd 0.05
rosparam set /lyapunov/k1 1.0
rosparam set /lyapunov/k2 0.1
rosservice call /controller/reset/params
python pub_init_pose.py -1.7 -0.59 0
