#ifndef ROVER_CONTROL_H
#define ROVER_CONTROL_H

#include "Socket.h"

class RoverControl {
    public:
        RoverControl();

        void update();

    private:
        UDPSocket socket;
        float left_motor;
        float right_motor;
};

#endif // ROVER_CONTROL_H
