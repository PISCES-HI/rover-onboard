#ifndef SERIAL_H
#define SERIAL_H

#include <stddef.h>
#include <termios.h>

class Serial {
    public:
        Serial(const char* port_name, int baud = B9600);

        size_t read_buffer(char* buffer, size_t len);
        size_t write_buffer(const char* buffer, size_t len);
        size_t writeln(const char* line);

    private:
        int set_interface_attribs(int speed, int parity);
        void set_blocking(bool should_block);

        int fd;
};

#endif // SERIAL_H
