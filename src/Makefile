OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

INC_DIRS = /usr/include/ni /usr/include/nite \
    ../lib/gl ../lib/glh ../lib/glut

SRC_FILES = main.cpp SceneDrawer.cpp

EXE_NAME = Lumen

USED_LIBS += glut OpenNI GLU XnVNite_1_5_0
CFLAGS += -lml -lcvaux -lhighgui -lcv -lcxcore

LIB_DIRS += /usr/local/lib

include ../lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Samples/Build/Common/CommonCppMakefile

