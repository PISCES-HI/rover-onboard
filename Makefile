CC=g++
CFLAGS=-I./include/
LDFLAGS=-L/usr/local/lib -ldln

all: dlntest

dlntest: main.o
	$(CC) $(LDFLAGS) main.o -o dlntest

main.o: main.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) main.cpp
    	
clean:
	rm *.o dlntest
