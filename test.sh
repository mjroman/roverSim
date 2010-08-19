#!/bin/sh
if [ ! -e ./Makefile ]
then
	echo "Running Qmake"
	qmake 
fi

make -s && open build/QtRoverSimulator_debug.app
