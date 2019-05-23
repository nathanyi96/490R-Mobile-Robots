#!/bin/bash
map_idx=0

for k1 in 0.1 1 5 10
do
	for k2 in 0.1 1 5 10
	do
		for init_pose in -1.6,0.2,0.7 -1.6,0.2,1.57 -1.6,-1.0,-0.5 -1.6,-1.0,-1.57 
		do
			IFS=',' read x y theta <<< "${init_pose}"
			echo "experiment with param $k1 $k2, initial pose $x $y $theta"
			rosparam set /lyapunov/k1 $k1
			rosparam set /lyapunov/k1 $k2
			rosservice call /controller/reset/params
			python runner_script_log.py $map_idx
			python pub_init_pose.py $x $y $theta
			sleep 15
		done
	done		
done

