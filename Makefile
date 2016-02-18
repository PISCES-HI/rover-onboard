CC=g++
CFLAGS=-I./src/ -I./thirdparty/include
LDFLAGS=-L/usr/local/lib -L/usr/local/Trolltech/Qt-4.8.7/lib/ -ldln -lQtCore
SRC_OBJS=analog.o i2c.o PwmDriver.o Socket.o RoverControl.o
OBJS=main.o $(SRC_OBJS)

all: bin/rover_onboard

bin/rover_onboard: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o bin/rover_onboard

main.o: src/main.cpp $(SRC_OBJS)
	$(CC) $(CFLAGS) -c src/main.cpp

analog.o: src/analog.cpp src/analog.h
	$(CC) $(CFLAGS) -c src/analog.cpp

i2c.o: src/i2c.cpp src/i2c.h
	$(CC) $(CFLAGS) -c src/i2c.cpp

PwmDriver.o: src/PwmDriver.cpp src/PwmDriver.h
	$(CC) $(CFLAGS) -c src/PwmDriver.cpp

Socket.o: src/Socket.cpp src/Socket.h
	$(CC) $(CFLAGS) -c src/Socket.cpp

RoverControl.o: src/RoverControl.cpp src/RoverControl.h src/util.h
	$(CC) $(CFLAGS) -c src/RoverControl.cpp
    	
clean:
	rm *.o bin/rover_onboard
