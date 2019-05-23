#!/bin/bash
map_idx=0
L=1.0
for V in 1.0 1.5 2.0
do
	for R in 1.5 2.5 3.0 4.0 5.0 10.0
	do
		for init_pose in -1.6,-1.0,-0.5
		do
			IFS=',' read x y theta <<< "${init_pose}"
			echo "experiment with param $L, $R, $V initial pose $x $y $theta"
			rosparam set /purepursuit/pose_lookahead $L
			rosservice call /controller/reset/params
			python runner_script_log.py $map_idx $V $R
			python pub_init_pose.py $x $y $theta
			sleep 15
		done
	done		
done

