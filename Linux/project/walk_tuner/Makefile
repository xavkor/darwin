###############################################################
#
# Purpose: Makefile for "walk_tuner"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = walk_tuner

INCLUDE_DIRS = -I../../include -I../../../Framework/include

CXX = g++

CXXFLAGS +=	-O2 -DLINUX -g -Wall -D_REENTRANT -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lncurses -lrt -ljpeg 

OBJS =	./cmd_process.o	\
		./main.o


all: darwin.a $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): $(OBJS) ../../lib/darwin.a
	#$(CXX) -o $(TARGET) $(OBJS) `pkg-config --cflags opencv` ../../lib/darwin.a $(LIBS) 
	$(CXX) $(CFLAGS) $(OBJS) ../../lib/darwin.a -o $(TARGET) $(LIBS) `pkg-config --libs opencv`

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/walk_tuner_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
