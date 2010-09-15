#!/bin/sh
if [ ! -e ./Makefile ]
then
	echo "Running Qmake"
	qmake 
fi

make -s && open build/QtRoverSimulator_debug.app
if [  -f ../sounds/woman-phaseComplete.mp3 ]
then
	afplay ../sounds/woman-phaseComplete.mp3
fi
