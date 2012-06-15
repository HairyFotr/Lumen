OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

INC_DIRS = /usr/include/opencv2 /usr/include/ni /usr/include/nite /usr/include

SRC_FILES = Geometry.cpp LumenRender.cpp main.cpp Utils.cpp

EXE_NAME = Lumen

USED_LIBS += glut OpenNI GLU XnVNite_1_5_2
CFLAGS += -lml -lcvaux -lhighgui -lcv -lcxcore -std=gnu++0x -Wall
LDFLAGS += `pkg-config --libs opencv` -lboost_thread-mt

LIB_DIRS += /usr/lib

include ../lib/OpenNI/Samples/Build/Common/CommonCppMakefile
