#include "RoverControl.h"

#include <iostream>
#include <stdint.h>
#include <string>

RoverControl::RoverControl() : socket("0.0.0.0", 30001) {
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
                break;
            case 'B':
                // Right motor
                break;
            case 'H':
                // LR motor
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
