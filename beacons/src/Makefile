CC = g++
STD = -std=c++11
USER_LIB = beacon.h
C_FLAGS = -Werror -Wall -pedantic
D_FLAGS = -g
CPP_FILES = main.cpp
APP = beacons

release: beacons
	$(CC) $(STD) $(C_FLAGS) $(CPP_FILES) beacons.o -o $(APP)

debug: beacons
	$(CC) $(STD) $(C_FLAGS) $(D_FLAGS) $(CPP_FILES) -o $(APP)

beacons:
	$(CC) $(STD) $(C_FLAGS) beacons.cpp -c

clean:
	rm -rf *.o beacons

.PHONY: clean 


