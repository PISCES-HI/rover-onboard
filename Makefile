CC=g++
CFLAGS=-I./src/ -I./thirdparty/include -std=c++11
LDFLAGS=-L/usr/local/lib -L/usr/local/Trolltech/Qt-4.8.7/lib/ -ldln -lQtCore
SRC_OBJS=AdxlDriver.o analog.o Hmc5883lDriver.o i2c.o PwmDriver.o Socket.o RoverControl.o thermistor.o
OBJS=main.o $(SRC_OBJS)

all: bin/rover_onboard

bin/rover_onboard: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o bin/rover_onboard

main.o: src/main.cpp $(SRC_OBJS)
	$(CC) $(CFLAGS) -c src/main.cpp

AdxlDriver.o: src/AdxlDriver.cpp src/AdxlDriver.h
	$(CC) $(CFLAGS) -c src/AdxlDriver.cpp

analog.o: src/analog.cpp src/analog.h
	$(CC) $(CFLAGS) -c src/analog.cpp

Hmc5883lDriver.o: src/Hmc5883lDriver.cpp src/Hmc5883lDriver.h
	$(CC) $(CFLAGS) -c src/Hmc5883lDriver.cpp

i2c.o: src/i2c.cpp src/i2c.h
	$(CC) $(CFLAGS) -c src/i2c.cpp

PwmDriver.o: src/PwmDriver.cpp src/PwmDriver.h
	$(CC) $(CFLAGS) -c src/PwmDriver.cpp

Socket.o: src/Socket.cpp src/Socket.h
	$(CC) $(CFLAGS) -c src/Socket.cpp

RoverControl.o: src/RoverControl.cpp src/RoverControl.h src/analog.h src/thermistor.h src/util.h
	$(CC) $(CFLAGS) -c src/RoverControl.cpp

thermistor.o: src/thermistor.cpp src/thermistor.h
	$(CC) $(CFLAGS) -c src/thermistor.cpp
    	
clean:
	rm *.o bin/rover_onboard
