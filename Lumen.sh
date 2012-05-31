#!/bin/bash

echo "Password is lumen :)"
sudo echo "LUMEN!"

#if [ "$1" == "build" ]
#then
	cd src
	make clean
	cd ..
	cd splash
	rm LumenSplash
	cd ..
#fi

out="0"
if [ ! -f bin/x86-Release/Lumen ]
then
	cd src
	make
	out="$?"
	cd ..
	if [ "$out" != "0" ]
	then
	    exit
    fi
	cd splash
	make
	cd ..
fi

cnt=0
cnt2=0 #reboot count

runLumen() {
    #echo "Password is lumen :)"
	hidraw="/dev/$( realpath /sys/class/hidraw/hidraw* | grep 014B | grep -o hidraw[0-9]$ )"
	sudo chmod +r $hidraw
	if [ "$?" == "0" ]
	then
	    ./Lumen $hidraw #2> /dev/null
	    if [ "$?" != "1" ]
	    then
            let "cnt+=1"
            echo cnt
            if [ $cnt -gt 5 ]
            then
                cnt=0
	            openbox --restart
	            #clear
	            #echo "If you can read this, and it's been here for a while, please reset the computer."
	            #echo " press ctrl-alt-f1, write lumen enter lumen then ctrl+alt+del :)"
	            let "cnt2+=1"
	            if [ $cnt2 -gt 5 ] 
	            then
	                echo "Rebooting in 5 seconds..."
	                sleep 6
	                sudo reboot
                fi
	            sleep 3
            fi
	        runLumen
        else
            cnt2=0
	    fi
	else
	    runLumen
	fi
}

#while :
#do
    # Run lumen
	cd bin/x86-Release
	runLumen
	cd ../..

    # Splash screen
    #cd splash
    #./LumenSplash
    #if [ "$?" == "77" ]
    #then
    #   exit
    #fi
    #cd ..
#done

