#include "RoverControl.h"

#include <iostream>
#include <cstdio>
#include <stdint.h>
#include <string>

RoverControl::RoverControl(HDLN& handle) : socket("0.0.0.0", 30001), l_motor(0.0), r_motor(0.0) {
    pwm.begin(handle);
    pwm.set_pwm_freq(handle, 50);
    pwm.set_pin(handle, 0, 512);
}

void RoverControl::update() {
    try {
        // Read a datagram from a client on the server socket
        uint8_t buffer[64];
        std::string source_addr;
        uint16_t source_port;
        this->socket.recvFrom(buffer, 64, source_addr, source_port);

        if ((char*)buffer == "connect me plz") {
            std::cout << source_addr << ":" << source_port << " has connected.\n";
            return;
        }

        char packet_id = buffer[0];
        switch (packet_id) {
            case 'A':
                // Left motor
                sscanf((char*)buffer+1, "%f|", &this->l_motor);
                std::cout << "Got l motor speed: " << this->l_motor << std::endl;
                break;
            case 'B':
                // Right motor
                sscanf((char*)buffer+1, "%f|", &this->r_motor);
                std::cout << "Got r motor speed: " << this->r_motor << std::endl;
                break;
            case 'H':
                // LR motor
                sscanf((char*)buffer+1, "%f|%f|", &this->l_motor, &this->r_motor);
                std::cout << "Got lr motor speed: " << this->l_motor << " " << this->r_motor << std::endl;
                break;
            case 'C':
                // Pan
                break;
            case 'D':
                // Tilt
                break;
            case 'E':
                // SADL
                break;
            case 'F':
                // Blade
                break;
            case 'G':
                // Brake
                break;
            case 'Z':
                // Text command
                break;
        }
    } catch (SocketException e) {
        std::cout << "Failed to receive on socket: " << e.what() << std::endl;
    }
}
