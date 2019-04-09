ifeq ($(OPTIMIZE), 1)
    FLG += -O2
else
    FLG += -g -O0
endif

trial: constraint_set.o crc.o protocol.o main.o
	g++ constraint_set.o crc.o protocol.o main.o -o trial `pkg-config --libs opencv4 eigen3` -pthread

constraint_set.o: constraint_set.cpp
	g++ $(FLG) -std=c++11 -c constraint_set.cpp `pkg-config --cflags opencv4 eigen3`

main.o: main.cpp
	g++ $(FLG) -std=c++11 -c main.cpp `pkg-config --cflags opencv4`

protocol.o: protocol.cpp
	g++ $(FLG) -std=c++11 -c protocol.cpp

crc.o: crc.c
	gcc $(FLG) -c crc.c

main.cpp: constraint_set.h protocol.h

constraint_set.cpp: constraint_set.h

protocol.cpp: protocol.h

.PHONY: clean
clean:
	rm -rf trial *.o
