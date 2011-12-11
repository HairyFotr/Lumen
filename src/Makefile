OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

INC_DIRS = /usr/include/ni /usr/include/nite \
    ../lib/gl ../lib/glh ../lib/glut 
#    ../lib/opencv/include/opencv2 \
#    ../lib/opencv/include/ \

#../lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Include \
#    ../lib/nite-bin-linux-x86-v1.5.0.2/Include \

SRC_FILES = main.cpp SceneDrawer.cpp

EXE_NAME = Lumen

USED_LIBS += glut OpenNI GLU XnVNite_1_5_0
#OpenCV
#CFLAGS += -lopencv_legacy -lopencv_core -lopencv_highgui

#-lml -lcvaux -lhighgui -lcv -lcxcore -pg -lstdc++
LIB_DIRS += /usr/local/lib \
    #/usr/include/opencv \
    #../lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Lib

include ../lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Samples/Build/Common/CommonCppMakefile

