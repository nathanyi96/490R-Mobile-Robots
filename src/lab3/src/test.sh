#!/bin/bash
#python run.py --map '../maps/map1.txt' -s 0 0 -g 5 4 --num-vertices 30 --connection-radius 100
#python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 200 --connection-radius 100
#python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 250 --connection-radius 100 --lazy
#python run.py -s 0 0 -g 5 5 --num-vertices 30
#python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 200 --connection-radius 100
#python runDubins.py -m ../maps/map2.txt -s 321 148 0 -g 106 202 90 --num-vertices 200 --connection-radius 100
python runDubins.py -m ../maps/map2.txt -s 30 30 90 -g 300 30 90 -c 0.1 --num-vertices 200 --connection-radius 150 --lazy

#python runDubins.py -m ../maps/map1.txt -c 5 -s 0 0 0 -g 5 4 90 --num-vertices 30 --lazy
