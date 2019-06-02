#!/bin/bash
#python run.py --map '../maps/map1.txt' -s 8 10 -g 5 4 --num-vertices 30 --connection-radius 20
#python run.py --map '../maps/map1.txt' -s 7 9 -g 5 4 --num-vertices 100 --connection-radius 30 --lazy
#python run.py -m ../maps/map1.txt -s 0 0 -g 8 7 --num-vertices 15
#python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 200 --connection-radius 100
#python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 250 --connection-radius 100 --lazy
#python run.py -s 0 0 -g 5 5 --num-vertices 30
python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 250 --connection-radius 100 --shortcut
#python runDubins.py -m ../maps/map2.txt -s 321 148 0 -g 106 202 90 --num-vertices 250 --connection-radius 100
#python runDubins.py -m ../maps/map2.txt -s 30 30 90 -g 300 300 90 -c 0.1 --num-vertices 200 --connection-radius 150 --lazy
#python runDubins.py -m ../maps/map1.txt -c 1.5 -s 0 0 0 -g 8 7 90 --num-vertices 30 --lazy
=======
#python run.py --map '../maps/map1.txt' -s 0 0 -g 5 4 --num-vertices 30 --connection-radius 100
#python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 200 --connection-radius 100
#python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 250 --connection-radius 100 --lazy
#python run.py -s 0 0 -g 5 5 --num-vertices 30
python run.py --map '../maps/map2.txt' -s 10 10 -g 350 350 --num-vertices 2000 --connection-radius 100
#python runDubins.py -m ../maps/map2.txt -s 321 148 0 -g 106 202 90 --num-vertices 200 --connection-radius 100
#python runDubins.py -m ../maps/map2.txt -s 30 30 90 -g 300 300 90 -c 0.1 --num-vertices 200 --connection-radius 150 --lazy
#python runDubins.py -m ../maps/map1.txt -c 5 -s 0 0 0 -g 5 4 90 --num-vertices 30 --lazy
