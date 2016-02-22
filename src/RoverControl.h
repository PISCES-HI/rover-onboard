#ifndef ROVER_CONTROL_H
#define ROVER_CONTROL_H

#include <ctime>
#include <string>

#include "dln/dln_generic.h"

#include "PwmDriver.h"
#include "Socket.h"

enum Command {
    CMD_FWD,
    CMD_REV,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_NONE,
};

struct Timer {
    // c is cooldown in seconds
    Timer(double c);
    bool tick();

    std::clock_t last_time;
    std::clock_t cooldown;
};

class RoverControl {
    public:
        RoverControl(HDLN& _handle);

        void update();

    private:
        void update_telemetry();
        void update_command();

        void set_cam_pan(float angle);
        void set_cam_tilt(float angle);
        void set_l_motor(int power);
        void set_r_motor(int power);
        void set_sadl(int power);
        void set_blade(int power);
        void set_brake(bool on);

        HDLN& handle;
        UDPSocket socket;
        PwmDriver pwm;
        float fwd_cam_pan;
        float fwd_cam_tilt;
        int l_motor;
        int r_motor;
        int sadl;
        int blade;
        bool brake;

        Timer tele_packet_timer; // Telemetry packet timer
        std::string telemetry_bundle; // Bundle of telemetry packets

        Timer tele_timer;

        Command cmd; // Command currently being executed, if any
        std::clock_t cmd_start; // Command timer
        double cmd_duration; // Command duration in seconds
};

#endif // ROVER_CONTROL_H
