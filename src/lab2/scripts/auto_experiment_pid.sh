#!/bin/bash
map_idx=0

for kp in 0.15 #0.3 1.0 3.0
do
	for kd in 0.2 #0.5 1.0 2.0
	do
		for init_pose in -1.6,-1.0,-1.57 #-1.6,0.2,0.7 -1.6,0.2,1.57 -1.6,-1.0,-0.5
		do
			IFS=',' read x y theta <<< "${init_pose}"
			echo "experiment with param $kp $kd, initial pose $x $y $theta"
			rosparam set /pid/kp $kp
			rosparam set /pid/kd $kd
			rosservice call /controller/reset/params
			python runner_script_log.py $map_idx
			python pub_init_pose.py $x $y $theta
			sleep 15
		done
	done		
done

