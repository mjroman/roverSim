#!/bin/sh
if [ ! -e ./Makefile ]
then
	echo "Running Qmake"
	qmake 
fi

make && open build/QtRoverSimulator_debug.app
