#ifndef ROVER_CONTROL_H
#define ROVER_CONTROL_H

#include "dln/dln_generic.h"

#include "PwmDriver.h"
#include "Socket.h"

class RoverControl {
    public:
        RoverControl(HDLN& handle);

        void update();

    private:
        UDPSocket socket;
        PwmDriver pwm;
        float l_motor;
        float r_motor;
        float fwd_cam_pan;
        float fwd_cam_tilt;
};

#endif // ROVER_CONTROL_H
