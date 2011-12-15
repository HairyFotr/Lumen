#!/bin/bash

if [ "$1" == "build" ]
then
	cd src
	make clean
	cd ..
fi

out="0"
if [ ! -f bin/x86-Release/Lumen ]
then
	cd src
	make
	out="$?"
	cd ..
fi
if [ "$out" == "0" ]
then
	cd bin/x86-Release
	./Lumen
fi
