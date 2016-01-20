CC=g++
CFLAGS=-I./src/ -I./thirdparty/include
LDFLAGS=-L/usr/local/lib -L/usr/local/Trolltech/Qt-4.8.7/lib/ -ldln -lQtCore
OBJS=main.o i2c.o PwmDriver.o

all: bin/rover_onboard

bin/rover_onboard: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o bin/rover_onboard

main.o: src/main.cpp
	$(CC) $(CFLAGS) -c src/main.cpp

i2c.o: src/i2c.cpp src/i2c.h
	$(CC) $(CFLAGS) -c src/i2c.cpp

PwmDriver.o: src/PwmDriver.cpp src/PwmDriver.h
	$(CC) $(CFLAGS) -c src/PwmDriver.cpp
    	
clean:
	rm *.o bin/rover_onboard
