ifeq ($(OPTIMIZE), 1)
    FLG += -O2
else
    FLG += -g -O0
endif

.PHONY: clean all

all: trial calibration picker

picker: picker.cpp
	g++ $(FLG) `pkg-config --cflags --libs opencv4` picker.cpp -o picker

trial: ng.o crc.o protocol.o main.o
	g++ ng.o crc.o protocol.o main.o -o trial `pkg-config --libs opencv4 eigen3` -pthread

calibration: calibration.cpp
	g++ $(FLG) -std=c++11 calibration.cpp -o calibration `pkg-config --cflags --libs opencv4`

ng.o: ng.cpp
	g++ $(FLG) -std=c++11 -c ng.cpp `pkg-config --cflags opencv4 eigen3`

main.o: main.cpp
	g++ $(FLG) -std=c++11 -c main.cpp `pkg-config --cflags opencv4`

protocol.o: protocol.cpp
	g++ $(FLG) -std=c++11 -c protocol.cpp

crc.o: crc.c
	gcc $(FLG) -c crc.c

main.cpp: ng.h protocol.h

ng.cpp: ng.h

protocol.cpp: protocol.h

clean:
	rm -rf calibration trial picker *.o
