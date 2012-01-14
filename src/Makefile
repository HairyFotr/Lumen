OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

INC_DIRS = /usr/include/opencv2 /usr/include/ni /usr/include/nite /usr/include

SRC_FILES = Geometry.cpp main.cpp LumenRender.cpp Utils.cpp

EXE_NAME = Lumen

USED_LIBS += glut OpenNI GLU XnVNite_1_5_2
CFLAGS += -Wall -lml -lcvaux -lhighgui -lcv -lcxcore
LDFLAGS += `pkg-config --libs opencv` -lboost_thread-mt

LIB_DIRS += /usr/lib

include ../lib/OpenNI-Bin-Dev-Linux-x86-v1.5.2.7/Samples/Build/Common/CommonCppMakefile
