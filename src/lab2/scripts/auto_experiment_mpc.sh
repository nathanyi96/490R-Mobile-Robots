#!/bin/bash
map_idx=0

for K in 60 140
do
	for T in 3 5 7
	do
		for init_pose in -1.6,0.2,0.7 -1.6,0.2,1.57 -1.6,-1.0,-0.5 -1.6,-1.0,-1.57 
		do
			IFS=',' read x y theta <<< "${init_pose}"
			echo "experiment with param $K $T, initial pose $x $y $theta"
			rosparam set /mpc/T $T
			rosparam set /mpc/K $K
			rosservice call /controller/reset/params
			python runner_script_log.py $map_idx
			python pub_init_pose.py $x $y $theta
			sleep 15
		done
	done		
done

