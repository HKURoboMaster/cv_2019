ifeq ($(OPTIMIZE), 1)
    FLG += -O2
else
    FLG += -g -O0
endif

trial: constraint_set.o main.o
	g++ constraint_set.o main.o -o trial `pkg-config --libs opencv eigen3`

constraint_set.o: constraint_set.cpp
	g++ $(FLG) -std=c++11 -c constraint_set.cpp `pkg-config --cflags opencv eigen3`

main.o: main.cpp
	g++ $(FLG) -std=c++11 -c main.cpp `pkg-config --cflags opencv`

main.cpp: constraint_set.h common.h

constraint_set.cpp: constraint_set.h common.h

.PHONY: clean
clean:
	rm -rf trial *.o
