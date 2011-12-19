OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

INC_DIRS = /usr/include/ni /usr/include/nite \
    /usr/include/GL /usr/local/include
    #/home/lumen/opencv/include 

SRC_FILES = main.cpp SceneDrawer.cpp

EXE_NAME = Lumen

USED_LIBS += glut OpenNI GLU XnVNite_1_5_0
CFLAGS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann
CFLAGS += -lml -lcvaux -lhighgui -lcv -lcxcore -lfuckyou

LIB_DIRS += /usr/local/lib /usr/local/lib

include ../lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Samples/Build/Common/CommonCppMakefile
