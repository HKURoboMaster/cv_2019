ifeq ($(shell pkg-config --cflags --libs opencv4 2>/dev/null),)
    $(error Unable to locate OpenCV 4)
endif
CXXFLAGS += -std=c++11
ifneq ($(shell which nvcc 2>/dev/null),)
    CXXFLAGS += -DCUDA
endif
ifeq ($(OPTIMIZE), 1)
    CFLAGS += -O2
    CXXFLAGS += -O2
else
    CFLAGS += -g -Og
    CXXFLAGS += -g -Og
endif
CXXFLAGS += $(shell pkg-config --cflags opencv4)
LDFLAGS += $(shell pkg-config --libs opencv4)
.PHONY: clean all

all: trial calibration picker

picker: picker.cpp
	$(CXX) $(CXXFLAGS) picker.cpp -o picker $(LDFLAGS)

trial: ng.o crc.o protocol.o main.o
	$(CXX) ng.o crc.o protocol.o main.o -o trial -pthread $(LDFLAGS)

calibration: calibration.cpp
	$(CXX) $(CXXFLAGS) calibration.cpp -o calibration $(LDFLAGS)

ng.o: ng.cpp
	$(CXX) $(CXXFLAGS) -c ng.cpp

main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp

protocol.o: protocol.cpp
	$(CXX) $(CXXFLAGS) -c protocol.cpp

crc.o: crc.c
	$(CC) $(CFLAGS) -c crc.c

main.cpp: ng.h protocol.h

ng.cpp: ng.h

protocol.cpp: protocol.h

clean:
	rm -rf calibration trial picker *.o *.dSYM
