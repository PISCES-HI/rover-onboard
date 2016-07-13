#include "Serial.h"

#include <cstring>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

Serial::Serial(const char* port_name, int baud) {
    fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "error " << errno << " opening " << port_name << ": " << strerror(errno) << std::endl;
        return;
    }

    set_interface_attribs(baud, 0);     // set speed to <baud> bps, 8n1 (no parity)
    set_blocking(false);                // set no blocking
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

size_t Serial::read_buffer(char* buffer, size_t len) {
    ssize_t n = read(fd, buffer, len);
    if (n == -1) { // Error occurred
        //std::cerr << "Error reading serial\n";
    }
    return n;
}

size_t Serial::write_buffer(const char* buffer, size_t len) {
    ssize_t n = write(fd, buffer, len);
    if (n == -1) { // Error occurred
        // std::cerr << "Error writing serial\n";
    }
    return n;
}

size_t Serial::writeln(const char* line) {
    size_t n = this->write_buffer(line, strlen(line));
    //size_t n_newline = this->write_buffer("\n", 1);
    return n /*+ n_newline*/;
}

int Serial::set_interface_attribs(int speed, int parity) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        std::cerr << "error " << errno << " from tcgetattr\n";
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    //cfmakeraw(&tty);

    // Flush Port, then applies attributes
    //tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "error " << errno << " from tcsetattr\n";
        return -1;
    }
    return 0;
}

void Serial::set_blocking(bool should_block) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        std::cerr << "error " << errno << " from tggetattr\n";
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        std::cerr << "error " << errno << " setting term attributes\n";
}
