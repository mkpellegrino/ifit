# 17 JUN 2020 - 00 28
# Be sure to change the name of the program here
FILE_NAME=serial-comms
PATH=/usr/local/bin:/opt/local/bin:/usr/bin:./
INCLUDE=-I/usr/local/include/c++/10.1.0/
XINCLUDE=-I/opt/X11/include
XLIBS=-L/opt/X11/lib/ -lX11
DEBUG=-g -DDEBUG
OPT=-O3
#CXX=g++
#CC=gcc
CXX=/usr/local/gfortran/bin/g++
CC=/usr/local/gfortran/bin/gcc
RM=/bin/rm


BIN_DIR=./
LIB_DIR=./
LST_DIR=./


release	: serial-comms.cpp Makefile
	$(value CXX) $(OPT) serial-comms.cpp $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE) $(LDFLAGS) -o $(BIN_DIR)serial-comms
	@echo "Done."

debug	: serial-comms.cpp Makefile
	$(value CXX) $(DEBUG) serial-comms.cpp $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE) $(LDFLAGS) -o $(BIN_DIR)serial-comms-debug
	@echo "Done."

ai	: csafe-ai.c Makefile
	$(value CC) csafe-ai.c $(CFLAGS) $(INCLUDE) $(LDFLAGS) -o $(BIN_DIR)csafe-ai
	@echo "Done."

decode	: decode.c
	$(CC) decode.c $(CFLAGS) $(INCLUDE) $(LDFLAGS) -o $(BIN_DIR)decode
	@echo "Done."

all	: clean release ai debug

verbose: all
	@echo ""
	@echo "C++ Compiler (CXX): " $(value CXX) 
	@echo "C Compiler (CC): " $(value CC)
	@echo "C Preprocessor: " $(value CPP)
	@echo ""
	$(value CC) --version
	@echo ""
	@echo "Environment Variables:"
	@echo " BIN_DIR: " $(value BIN_DIR)
	@echo " LIB_DIR: " $(value LIB_DIR)
	@echo " INCLUDE: " $(value INCLUDE)
	@echo " LST_DIR: " $(value LST_DIR)
	@echo "     OPT: " $(value OPT)
	@echo "CXXFLAGS: " $(value CXXFLAGS)
	@echo "CPPFLAGS: " $(value CPPFLAGS)
	@echo " LDFLAGS: " $(value LDFLAGS)
	@echo "   DEBUG: " $(value DEBUG)
	@echo "XINCLUDE: " $(value XINCLUDE)
	@echo "   XLIBS: " $(value XLIBS)
	@echo ""

	make all
	@echo "Build Completed"

tidy:
	$(RM) -fR $(LIB_DIR)*.o
	$(RM) -f *~
	$(RM) -f *.*~
	@echo "Tidied up a bit."

clean: tidy
	$(RM) -fR $(BIN_DIR)serial-comms
	$(RM) -fR $(BIN_DIR)serial-comms-debug
	$(RM) -fR $(BIN_DIR)serial-comms-debug.dSYM
	$(RM) -fR $(BIN_DIR)csafe-ai-debug.dSYM
	$(RM) -fR $(BIN_DIR)csafe-ai-debug

help:
	@echo "Makefile - by: Michael K. Pellegrino"
	@echo "usage: make all|release|debug|clean|tidy|verbose"
	@echo ""
	@echo "Environment Variables:"
	@echo "CC: use export CC=/path/to/c/compiler"
	@echo "CPP: use export CPP=/path/to/c++/preprocessor"
	@echo "BIN_DIR"
	@echo "LIB_DIR"
	@echo "INCLUDE"
	@echo "LST_DIR"
	@echo "OPT"
	@echo "CXXFLAGS"
	@echo "CPPFLAGS"
	@echo "LDFLAGS"
	@echo "DEBUG"
	@echo "XINCLUDE"
	@echo "XLIBS"
