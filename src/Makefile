OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

INC_DIRS = ../lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Include \
    ../lib/nite-bin-linux-x86-v1.5.0.2/Include \
    /usr/include/ni \
    /usr/include/opencv

SRC_FILES = main.cpp SceneDrawer.cpp

EXE_NAME = Lumen

USED_LIBS += glut OpenNI cv GLU
#OpenCV
CFLAGS += -lml -lcvaux -lhighgui -lcv -lcxcore -pg
LIB_DIRS += /usr/local/lib \
    ../lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Lib    

include ..lib/OpenNI-Bin-Dev-Linux-x86-v1.4.0.2/Samples/Build/Common/CommonCppMakefile

