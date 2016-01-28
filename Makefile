CC=g++
CFLAGS=-I./src/ -I./thirdparty/include
LDFLAGS=-L/usr/local/lib -L/usr/local/Trolltech/Qt-4.8.7/lib/ -ldln -lQtCore
OBJS=main.o i2c.o PwmDriver.o PracticalSocket.o

all: bin/rover_onboard

bin/rover_onboard: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o bin/rover_onboard

main.o: src/main.cpp src/PwmDriver.h src/PwmDriver.cpp src/i2c.h src/i2c.cpp
	$(CC) $(CFLAGS) -c src/main.cpp

i2c.o: src/i2c.cpp src/i2c.h
	$(CC) $(CFLAGS) -c src/i2c.cpp

PwmDriver.o: src/PwmDriver.cpp src/PwmDriver.h
	$(CC) $(CFLAGS) -c src/PwmDriver.cpp

PracticalSocket.o: src/PracticalSocket.cpp src/PracticalSocket.h
	$(CC) $(CFLAGS) -c src/PracticalSocket.cpp
    	
clean:
	rm *.o bin/rover_onboard
