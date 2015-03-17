###############################################################
#
# Purpose: Makefile for "face_head_tracking"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = face_head_tracking

INCLUDE_DIRS = -I../../../include -I../../../../Framework/include

CXX = g++
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt

OBJECTS =   main.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) `pkg-config --cflags opencv` ../../../lib/darwin.a -o $(TARGET) $(LFLAGS) `pkg-config --libs opencv`
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/face_head_tracking_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
